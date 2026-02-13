"""Conversation node: STT → LLM → TTS loop.

Triggered by wake word events. Runs the full conversation pipeline in a
background thread so the ROS executor is never blocked.

State machine:
  IDLE → (wake event) → GREETING → LISTENING_USER → PROCESSING → SPEAKING → LISTENING_USER → ...
  Any state → (timeout / goodbye) → IDLE
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from std_msgs.msg import Bool

from perseus_interfaces.msg import LEDCommand, VoiceEvent, VoiceState

from .interaction_logger import InteractionLogger
from .llm_client import OllamaClient
from .stt_engine import FasterWhisperSTT
from .tts_engine import PiperTTS


GOODBYE_PHRASES = {"goodbye", "bye", "see you", "that's all", "stop", "quit", "exit", "nevermind"}

GREETING_TEXT = "Hey there! What can I help you with?"
OLLAMA_ERROR_TEXT = (
    "Sorry, I can't reach my language model right now. "
    "Please make sure Ollama is running."
)
GOODBYE_TEXT = "Alright, see you later!"
TIMEOUT_TEXT = "I haven't heard anything, so I'll go back to sleep. Just say hey Perseus if you need me!"
ERROR_TEXT = "Sorry, something went wrong. Let me go back to listening mode."


class ConversationNode(Node):
    def __init__(self):
        super().__init__("conversation_node")

        # Parameters
        self.declare_parameter("ollama_url", "http://localhost:11434")
        self.declare_parameter("ollama_model", "llama3.2:1b")
        self.declare_parameter("stt_model_size", "tiny.en")
        self.declare_parameter("stt_device", "auto")
        self.declare_parameter("piper_model_path", "")
        self.declare_parameter("piper_bin", "piper")
        self.declare_parameter("silence_threshold", 500)
        self.declare_parameter("silence_duration", 2.0)
        self.declare_parameter("max_listen_duration", 30.0)
        self.declare_parameter("conversation_timeout", 30.0)
        self.declare_parameter("max_turns", 20)
        self.declare_parameter("wait_for_person_lock", True)
        self.declare_parameter("person_lock_timeout", 5.0)
        self.declare_parameter("interaction_log_path", "~/.ros/voice_interaction_log.json")

        # Publishers
        self._state_pub = self.create_publisher(VoiceState, "/voice/state", 10)
        self._led_pub = self.create_publisher(LEDCommand, "/voice/led_command", 10)
        self._transcript_pub = self.create_publisher(String, "/voice/user_transcript", 10)
        self._response_pub = self.create_publisher(String, "/voice/response_text", 10)

        # Subscribers
        self._wake_sub = self.create_subscription(
            VoiceEvent, "/voice/wake_event", self._wake_event_cb, 10
        )
        self._person_detected_sub = self.create_subscription(
            Bool, "/voice/person_detected", self._person_detected_cb, 10
        )

        # Person tracking state
        self._person_detected = False

        # Engines (lazy-loaded)
        self._stt = None
        self._tts = None
        self._llm = None

        # State
        self._state = VoiceState.IDLE
        self._turn_count = 0
        self._current_transcript = ""
        self._current_response = ""
        self._conversation_thread = None
        self._conversation_active = False

        # Interaction logger
        import os
        log_path = os.path.expanduser(
            self.get_parameter("interaction_log_path").value
        )
        self._interaction_logger = InteractionLogger(self, log_path=log_path)

        # Wake event data (stored for the conversation thread)
        self._wake_phrase = ""
        self._wake_confidence = 0.0

        # Publish initial idle state
        self._publish_state()

        self.get_logger().info("Conversation node started. Waiting for wake events...")

    def _init_engines(self):
        """Lazy-initialize STT, TTS, and LLM on first wake event."""
        if self._stt is None:
            stt_model = self.get_parameter("stt_model_size").value
            stt_device = self.get_parameter("stt_device").value
            self.get_logger().info(f"Loading STT model: {stt_model} (device: {stt_device})")
            self._stt = FasterWhisperSTT(
                model_size=stt_model,
                device=stt_device,
            )

        if self._tts is None:
            piper_model = self.get_parameter("piper_model_path").value
            piper_bin = self.get_parameter("piper_bin").value
            self._tts = PiperTTS(
                model_path=piper_model if piper_model else None,
                piper_bin=piper_bin,
            )
            if not self._tts.available:
                self.get_logger().warn("Piper TTS not available. Speech output disabled.")

        if self._llm is None:
            ollama_url = self.get_parameter("ollama_url").value
            ollama_model = self.get_parameter("ollama_model").value
            self._llm = OllamaClient(
                base_url=ollama_url,
                model=ollama_model,
            )

    def _wake_event_cb(self, msg):
        """Handle wake word detection."""
        if self._conversation_active:
            self.get_logger().debug("Ignoring wake event — conversation already active")
            return

        self.get_logger().info(
            f"Wake event received: {msg.wake_phrase} ({msg.confidence:.2f})"
        )

        self._wake_phrase = msg.wake_phrase
        self._wake_confidence = msg.confidence
        self._person_detected = False
        self._conversation_active = True
        self._conversation_thread = threading.Thread(
            target=self._conversation_loop, daemon=True
        )
        self._conversation_thread.start()

    def _person_detected_cb(self, msg):
        """Track whether the person tracker has locked on."""
        self._person_detected = msg.data

    def _conversation_loop(self):
        """Main conversation loop. Runs in a background thread."""
        record = self._interaction_logger.start_interaction(
            self._wake_phrase, self._wake_confidence
        )
        end_reason = "error"

        try:
            self._init_engines()
            self._llm.reset_history()
            self._turn_count = 0

            # Greeting — optionally wait for person tracker to lock on
            self._set_state(VoiceState.GREETING)

            wait_for_person = self.get_parameter("wait_for_person_lock").value
            if wait_for_person:
                lock_timeout = self.get_parameter("person_lock_timeout").value
                self.get_logger().info(
                    f"Waiting up to {lock_timeout:.1f}s for person tracker to lock..."
                )
                t0 = time.monotonic()
                while time.monotonic() - t0 < lock_timeout:
                    if self._person_detected:
                        self.get_logger().info("Person locked — facing user")
                        break
                    time.sleep(0.1)
                else:
                    self.get_logger().info("Person lock timed out — greeting anyway")
                record.person_track_duration_sec = time.monotonic() - t0
                record.person_detected = self._person_detected

            self.get_logger().info("Speaking greeting...")
            t0 = time.monotonic()
            self._speak(GREETING_TEXT)
            record.greeting_tts_duration_sec = time.monotonic() - t0

            # Check Ollama availability
            if not self._llm.is_available():
                self.get_logger().error(
                    "Ollama is not reachable. Install with: "
                    "curl -fsSL https://ollama.com/install.sh | sh && "
                    "sudo systemctl start ollama && "
                    "ollama pull llama3.2:1b"
                )
                self._speak(OLLAMA_ERROR_TEXT)
                end_reason = "ollama_unavailable"
                self._end_conversation(record, end_reason)
                return

            max_turns = self.get_parameter("max_turns").value
            timeout = self.get_parameter("conversation_timeout").value

            # Conversation turns
            while self._conversation_active and self._turn_count < max_turns:
                # Listen for user speech
                self._set_state(VoiceState.LISTENING_USER)
                self.get_logger().info("Listening for user speech...")

                t0 = time.monotonic()
                transcript = self._listen()
                stt_dur = time.monotonic() - t0

                if not transcript:
                    self.get_logger().info("No speech detected, timing out")
                    self._speak(TIMEOUT_TEXT)
                    end_reason = "timeout"
                    break

                self._current_transcript = transcript
                self._publish_transcript(transcript)
                self.get_logger().info(f"User said: {transcript}")

                # Check for goodbye
                if any(phrase in transcript.lower() for phrase in GOODBYE_PHRASES):
                    self.get_logger().info("Goodbye detected")
                    record.user_transcripts.append(transcript)
                    record.stt_durations_sec.append(stt_dur)
                    self._speak(GOODBYE_TEXT)
                    end_reason = "goodbye"
                    break

                # Process with LLM
                self._set_state(VoiceState.PROCESSING)
                self.get_logger().info("Processing with LLM...")

                t0 = time.monotonic()
                response = self._llm.chat(transcript)
                llm_dur = time.monotonic() - t0

                if response is None:
                    self.get_logger().warn("LLM returned no response")
                    record.user_transcripts.append(transcript)
                    record.stt_durations_sec.append(stt_dur)
                    record.llm_durations_sec.append(llm_dur)
                    self._speak(OLLAMA_ERROR_TEXT)
                    end_reason = "error"
                    break

                self._current_response = response
                self._publish_response(response)
                self.get_logger().info(f"LLM response: {response}")

                # Speak response
                self._set_state(VoiceState.SPEAKING)
                t0 = time.monotonic()
                self._speak(response)
                tts_dur = time.monotonic() - t0

                self._turn_count += 1

                # Record this turn
                record.user_transcripts.append(transcript)
                record.llm_responses.append(response)
                record.stt_durations_sec.append(stt_dur)
                record.llm_durations_sec.append(llm_dur)
                record.tts_durations_sec.append(tts_dur)
            else:
                # Loop ended because max_turns reached (not via break)
                if self._turn_count >= max_turns:
                    end_reason = "max_turns"

        except Exception as e:
            self.get_logger().error(f"Conversation loop error: {e}")
            self._speak(ERROR_TEXT)
            end_reason = "error"
        finally:
            self._end_conversation(record, end_reason)

    def _listen(self):
        """Record user speech and transcribe it."""
        from .audio_utils import record_until_silence

        silence_threshold = self.get_parameter("silence_threshold").value
        silence_duration = self.get_parameter("silence_duration").value
        max_duration = self.get_parameter("max_listen_duration").value

        try:
            audio = record_until_silence(
                silence_threshold=silence_threshold,
                silence_duration=silence_duration,
                max_duration=max_duration,
            )

            if len(audio) == 0:
                return ""

            return self._stt.transcribe(audio)
        except Exception as e:
            self.get_logger().error(f"Recording/transcription error: {e}")
            return ""

    def _speak(self, text):
        """Synthesize and play text."""
        if self._tts and self._tts.available:
            self._tts.speak(text)
        else:
            self.get_logger().warn(f"TTS unavailable, would say: {text}")
            time.sleep(1.0)

    def _set_state(self, state):
        """Update and publish voice state."""
        self._state = state
        self._publish_state()

        # Also send LED command for the state
        led_msg = LEDCommand()
        if state == VoiceState.IDLE:
            led_msg.mode = LEDCommand.OFF
        elif state == VoiceState.LISTENING_WAKE:
            led_msg.red, led_msg.green, led_msg.blue = 0, 20, 0
            led_msg.mode = LEDCommand.SOLID
        elif state == VoiceState.GREETING:
            led_msg.red, led_msg.green, led_msg.blue = 0, 0, 80
            led_msg.mode = LEDCommand.SOLID
        elif state == VoiceState.LISTENING_USER:
            led_msg.red, led_msg.green, led_msg.blue = 0, 0, 255
            led_msg.mode = LEDCommand.SOLID
        elif state == VoiceState.PROCESSING:
            led_msg.red, led_msg.green, led_msg.blue = 255, 165, 0
            led_msg.mode = LEDCommand.PULSE
        elif state == VoiceState.SPEAKING:
            led_msg.red, led_msg.green, led_msg.blue = 0, 255, 0
            led_msg.mode = LEDCommand.SOLID
        self._led_pub.publish(led_msg)

    def _publish_state(self):
        """Publish current voice state."""
        msg = VoiceState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = self._state
        msg.current_transcript = self._current_transcript
        msg.current_response = self._current_response
        msg.turn_count = self._turn_count
        self._state_pub.publish(msg)

    def _publish_transcript(self, text):
        """Publish user transcript."""
        msg = String()
        msg.data = text
        self._transcript_pub.publish(msg)

    def _publish_response(self, text):
        """Publish LLM response."""
        msg = String()
        msg.data = text
        self._response_pub.publish(msg)

    def _end_conversation(self, record=None, end_reason=""):
        """End the conversation and return to idle."""
        if record is not None:
            record.end_reason = end_reason
            self._interaction_logger.finish_interaction(record)
            self.get_logger().info(
                f"Interaction logged: {record.num_turns} turns, "
                f"{record.total_duration_sec:.1f}s total, reason={end_reason}"
            )

        self._conversation_active = False
        self._current_transcript = ""
        self._current_response = ""
        self._turn_count = 0
        self._set_state(VoiceState.LISTENING_WAKE)
        self.get_logger().info("Conversation ended. Returning to wake word listening.")

    def destroy_node(self):
        self._conversation_active = False
        if self._stt:
            self._stt.unload()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ConversationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
