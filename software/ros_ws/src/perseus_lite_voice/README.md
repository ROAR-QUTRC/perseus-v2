# Perseus Lite Voice

Local, fully-offline voice chat system for the Perseus Lite robot (Jetson Orin Nano 8GB). Detects the wake word "Hey Perseus", turns to face the nearest person, and runs a conversational STT/LLM/TTS loop — all without cloud dependencies.

## Features

- **Wake word detection** via OpenWakeWord (always-on, low CPU)
- **Speech-to-text** via Faster-Whisper (`tiny.en`, CUDA-accelerated)
- **LLM conversation** via Ollama (`llama3.2:1b`, local HTTP API)
- **Text-to-speech** via Piper (ONNX, CPU-based)
- **Person tracking** via YOLOv8n — detects nearest person on camera and rotates the robot to face them before greeting
- **RGB LED feedback** via PicoDev 3x RGB LED Module over I2C
- **Voice activity detection** with configurable silence thresholds

## Architecture

Three ROS2 nodes work together:

```
                      /voice/wake_event
[wake_word_node] ──────────────────────────→ [conversation_node]
  - Always-on mic listener                    - STT → LLM → TTS loop
  - OpenWakeWord detection                    - Publishes /voice/state
  - PicoDev LED control          ←──────────  - Publishes /voice/led_command
                                /voice/state
                                     │
                                     ▼
                             [person_tracker_node]
                               - YOLOv8n on /image_raw
                               - Proportional rotation
                               - Publishes cmd_vel_voice
                               - Publishes /voice/person_detected
```

### State Machine

```
IDLE ──(wake word)──→ GREETING ──→ LISTENING_USER ──→ PROCESSING ──→ SPEAKING ──┐
                         │              ▲                                        │
                         │              └────────────────────────────────────────┘
                         │
                    (person tracker activates,
                     rotates to face user,
                     then TTS greeting plays)

Any state ──(timeout / "goodbye")──→ IDLE
```

### LED Colors

| State                 | Color       | Mode  |
| --------------------- | ----------- | ----- |
| Idle                  | Off         | -     |
| Listening (wake word) | Dim green   | Solid |
| Greeting              | Blue        | Solid |
| Listening (user)      | Bright blue | Solid |
| Processing            | Orange      | Pulse |
| Speaking              | Green       | Solid |

## Topics

| Topic                        | Type                                  | Direction                     | Description                                            |
| ---------------------------- | ------------------------------------- | ----------------------------- | ------------------------------------------------------ |
| `/voice/wake_event`          | `perseus_interfaces/VoiceEvent`       | wake_word → conversation      | Wake word detected                                     |
| `/voice/state`               | `perseus_interfaces/VoiceState`       | conversation → all            | Current conversation state                             |
| `/voice/led_command`         | `perseus_interfaces/LEDCommand`       | conversation → wake_word      | LED color/mode control                                 |
| `/voice/user_transcript`     | `std_msgs/String`                     | conversation → any            | Transcribed user speech                                |
| `/voice/response_text`       | `std_msgs/String`                     | conversation → any            | LLM response text                                      |
| `/voice/person_detected`     | `std_msgs/Bool`                       | person_tracker → conversation | Person locked on                                       |
| `/voice/person_detection`    | `perseus_interfaces/PersonDetection`  | person_tracker → any          | Detection details (angle, bbox area)                   |
| `/voice/interaction_log`     | `perseus_interfaces/VoiceInteraction` | conversation → any            | Per-interaction timing and content record              |
| `/voice/interaction_summary` | `std_msgs/String`                     | conversation → any            | JSON summary of last 50 interactions + aggregate stats |
| `cmd_vel_voice`              | `geometry_msgs/Twist`                 | person_tracker → cmd_vel_mux  | Rotation commands                                      |

## Hardware Requirements

- **Microphone** — USB (e.g. Logitech C920 built-in mic)
- **Speaker** — USB or 3.5mm audio output
- **Camera** — USB webcam on `/image_raw` (for person tracking)
- **PicoDev 3x RGB LED Module** — I2C bus 1, default address `0x08` (optional, degrades gracefully)

## Prerequisites

### System Packages

```bash
# Audio I/O
sudo apt-get install alsa-utils portaudio19-dev

# I2C tools (for LED debugging)
sudo apt-get install i2c-tools
```

### Python Dependencies

```bash
pip install openwakeword faster-whisper piper-tts sounddevice numpy smbus2 ultralytics
```

### Ollama (LLM Backend)

```bash
# Install
curl -fsSL https://ollama.com/install.sh | sh

# Enable and start service
sudo systemctl enable ollama
sudo systemctl start ollama

# Pull model (~1.3GB)
ollama pull llama3.2:1b

# Verify
ollama run llama3.2:1b "Hello, who are you?"
```

### Piper TTS Voice Model

Download a voice from [Piper Voices](https://github.com/rhasspy/piper/blob/master/VOICES.md):

```bash
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json
```

Then set `piper_model_path` in `config/voice_params.yaml` to the `.onnx` file path.

## Building

```bash
colcon build --packages-select perseus_interfaces perseus_lite_voice
source install/setup.bash
```

## Usage

### Launch All Nodes

```bash
ros2 launch perseus_lite_voice voice.launch.py
```

### Launch Individual Nodes

```bash
# Wake word detection + LED
ros2 run perseus_lite_voice wake_word_node

# Conversation (STT/LLM/TTS)
ros2 run perseus_lite_voice conversation_node

# Person tracking (YOLO + rotation)
ros2 run perseus_lite_voice person_tracker_node
```

### Monitor

```bash
# Watch state transitions
ros2 topic echo /voice/state

# Watch transcriptions
ros2 topic echo /voice/user_transcript

# Watch LLM responses
ros2 topic echo /voice/response_text

# Watch person detection
ros2 topic echo /voice/person_detection
```

## Configuration

All parameters are in `config/voice_params.yaml` and can be overridden at launch.

### Wake Word Node

| Parameter              | Default      | Description                      |
| ---------------------- | ------------ | -------------------------------- |
| `wake_word_model`      | `hey_jarvis` | OpenWakeWord model name          |
| `confidence_threshold` | `0.5`        | Detection confidence (0-1)       |
| `sample_rate`          | `16000`      | Audio sample rate (Hz)           |
| `chunk_size`           | `1280`       | Audio chunk size (80ms at 16kHz) |
| `led_i2c_address`      | `0x08`       | PicoDev LED I2C address          |
| `led_i2c_bus`          | `1`          | I2C bus number                   |
| `led_brightness`       | `80`         | Global LED brightness (0-255)    |
| `led_pulse_rate_hz`    | `2.0`        | Pulse animation speed            |

### Conversation Node

| Parameter              | Default                             | Description                                  |
| ---------------------- | ----------------------------------- | -------------------------------------------- |
| `ollama_url`           | `http://localhost:11434`            | Ollama API endpoint                          |
| `ollama_model`         | `llama3.2:1b`                       | LLM model name                               |
| `stt_model_size`       | `tiny.en`                           | Faster-Whisper model                         |
| `stt_device`           | `auto`                              | STT device (`cuda`, `cpu`, `auto`)           |
| `piper_model_path`     | `""`                                | Path to Piper `.onnx` voice model            |
| `piper_bin`            | `piper`                             | Path to Piper binary                         |
| `silence_threshold`    | `500`                               | RMS threshold for silence detection          |
| `silence_duration`     | `2.0`                               | Seconds of silence before stopping recording |
| `max_listen_duration`  | `30.0`                              | Max recording time per turn (seconds)        |
| `conversation_timeout` | `30.0`                              | End conversation after this much silence     |
| `max_turns`            | `20`                                | Max conversation turns before ending         |
| `wait_for_person_lock` | `true`                              | Wait for person tracker before greeting      |
| `person_lock_timeout`  | `5.0`                               | Max wait time for person lock (seconds)      |
| `interaction_log_path` | `~/.ros/voice_interaction_log.json` | Path to persist interaction history          |

### Person Tracker Node

| Parameter              | Default         | Description                           |
| ---------------------- | --------------- | ------------------------------------- |
| `camera_topic`         | `/image_raw`    | Camera image topic                    |
| `camera_hfov_deg`      | `70.0`          | Camera horizontal field of view       |
| `yolo_model`           | `yolov8n.pt`    | YOLO model (auto-exports to TensorRT) |
| `yolo_confidence`      | `0.5`           | Detection confidence threshold        |
| `detection_rate_hz`    | `10.0`          | YOLO inference rate                   |
| `max_angular_vel`      | `0.5`           | Max rotation speed (rad/s)            |
| `angular_deadband_deg` | `5.0`           | Stop rotating when within this angle  |
| `proportional_gain`    | `1.5`           | P-controller gain for rotation        |
| `search_timeout`       | `5.0`           | Give up searching after this long     |
| `cmd_vel_topic`        | `cmd_vel_voice` | Output velocity topic                 |

## cmd_vel_mux Integration

The person tracker publishes rotation commands on `cmd_vel_voice`, which is registered in the velocity mux at priority 5:

| Source                    | Topic                           | Priority     |
| ------------------------- | ------------------------------- | ------------ |
| Navigation (Nav2)         | `cmd_vel_nav`                   | 1 (lowest)   |
| **Voice Person Tracking** | `cmd_vel_voice`                 | **5**        |
| Manual Controller         | `/diff_base_controller/cmd_vel` | 10 (highest) |

Manual control always overrides voice rotation. Voice rotation overrides autonomous navigation.

## Resource Usage (Jetson Orin Nano 8GB)

| Component                | Memory      | Notes                      |
| ------------------------ | ----------- | -------------------------- |
| OpenWakeWord             | ~50MB RAM   | Always-on, CPU             |
| Faster-Whisper (tiny.en) | ~200MB VRAM | Loaded on first wake, CUDA |
| YOLOv8n (TensorRT)       | ~200MB VRAM | Loaded on first wake, GPU  |
| Ollama (llama3.2:1b)     | ~1.5GB RAM  | CPU/GPU split              |
| Piper TTS                | ~100MB RAM  | CPU only                   |

YOLO and Whisper don't run simultaneously (YOLO during GREETING, Whisper during LISTENING), so peak VRAM usage is ~400MB.

## Package Structure

```
perseus_lite_voice/
├── config/
│   └── voice_params.yaml          # All tunable parameters
├── launch/
│   └── voice.launch.py            # Launches all 3 nodes
├── models/
│   └── README.md                  # Model download instructions
├── perseus_lite_voice/
│   ├── __init__.py
│   ├── wake_word_node.py          # Wake word + LED control
│   ├── conversation_node.py       # STT → LLM → TTS state machine
│   ├── interaction_logger.py      # Interaction ring buffer + JSON persistence
│   ├── person_tracker_node.py     # YOLO person detection + rotation
│   ├── picodev_led.py             # PicoDev 3x RGB LED I2C driver
│   ├── audio_utils.py             # Mic recording + aplay playback
│   ├── tts_engine.py              # Piper TTS wrapper
│   ├── stt_engine.py              # Faster-Whisper STT wrapper
│   └── llm_client.py              # Ollama HTTP client
├── resource/
│   └── perseus_lite_voice         # ament index marker
├── package.xml
├── setup.cfg
└── setup.py
```

## Interaction Log

Each completed conversation is logged with per-step timing data. The last 50 interactions are kept in a ring buffer, persisted to `~/.ros/voice_interaction_log.json`, and published on two topics:

- **`/voice/interaction_log`** (`VoiceInteraction`) — published once per completed interaction with wake word info, per-turn transcripts/responses, and timing for each pipeline step (person tracking, STT, LLM, TTS).
- **`/voice/interaction_summary`** (`std_msgs/String`) — JSON summary republished after each interaction, containing all 50 records (newest first) and aggregate stats (avg/min/max for STT, LLM, TTS, total duration, turns per interaction).

```bash
# Watch individual interactions
ros2 topic echo /voice/interaction_log

# Get the full summary
ros2 topic echo /voice/interaction_summary
```

The `end_reason` field indicates why the conversation ended: `"goodbye"`, `"timeout"`, `"error"`, `"max_turns"`, or `"ollama_unavailable"`.

## Custom Interfaces (in perseus_interfaces)

| Message            | Fields                                                                                                        | Purpose                                                                              |
| ------------------ | ------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| `VoiceEvent`       | stamp, wake_phrase, confidence                                                                                | Wake word detection event                                                            |
| `VoiceState`       | stamp, state, current_transcript, current_response, turn_count                                                | Conversation state (IDLE/LISTENING_WAKE/GREETING/LISTENING_USER/PROCESSING/SPEAKING) |
| `LEDCommand`       | red, green, blue, mode                                                                                        | LED control (SOLID/PULSE/OFF)                                                        |
| `PersonDetection`  | stamp, detected, angle_offset_rad, bounding_box_area, num_persons                                             | Person detection result                                                              |
| `VoiceInteraction` | start_time, end_time, wake_phrase, wake_confidence, person_detected, per-turn transcripts/timings, end_reason | Complete interaction record with timing                                              |

## Graceful Degradation

The system is designed to work with missing hardware:

- **No LED**: Logs a warning, continues without visual feedback
- **No camera**: Person tracker stays inactive, conversation proceeds without rotation
- **Ollama not running**: Speaks an error message, returns to idle
- **No Piper**: Logs responses to console instead of speaking
- **No person found**: Times out after 5s, greets anyway
