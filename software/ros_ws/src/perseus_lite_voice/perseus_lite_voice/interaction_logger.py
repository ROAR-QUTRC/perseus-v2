"""Interaction logger: ring buffer of recent voice interactions.

Records timing data for each pipeline step (person tracking, STT, LLM, TTS)
and conversation content. Persists to JSON and publishes ROS summaries.
"""

import json
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List

from builtin_interfaces.msg import Time
from rclpy.node import Node
from std_msgs.msg import String

from perseus_interfaces.msg import VoiceInteraction


DEFAULT_LOG_PATH = os.path.expanduser("~/.ros/voice_interaction_log.json")
MAX_RECORDS = 50


@dataclass
class InteractionRecord:
    """Data for one complete wake-up interaction."""

    start_time: float = 0.0  # monotonic
    end_time: float = 0.0

    # Wake word
    wake_phrase: str = ""
    wake_confidence: float = 0.0

    # Person tracking
    person_detected: bool = False
    person_track_duration_sec: float = 0.0

    # Per-turn parallel arrays
    user_transcripts: List[str] = field(default_factory=list)
    llm_responses: List[str] = field(default_factory=list)
    stt_durations_sec: List[float] = field(default_factory=list)
    llm_durations_sec: List[float] = field(default_factory=list)
    tts_durations_sec: List[float] = field(default_factory=list)

    # Greeting
    greeting_tts_duration_sec: float = 0.0

    # Set on finish
    end_reason: str = ""

    # Wall-clock timestamps for JSON/ROS msg (set by logger)
    start_wall: float = 0.0
    end_wall: float = 0.0

    @property
    def num_turns(self) -> int:
        return len(self.user_transcripts)

    @property
    def total_duration_sec(self) -> float:
        if self.end_time > 0 and self.start_time > 0:
            return self.end_time - self.start_time
        return 0.0

    def to_dict(self) -> dict:
        return {
            "start_wall": self.start_wall,
            "end_wall": self.end_wall,
            "wake_phrase": self.wake_phrase,
            "wake_confidence": self.wake_confidence,
            "person_detected": self.person_detected,
            "person_track_duration_sec": self.person_track_duration_sec,
            "user_transcripts": self.user_transcripts,
            "llm_responses": self.llm_responses,
            "stt_durations_sec": self.stt_durations_sec,
            "llm_durations_sec": self.llm_durations_sec,
            "tts_durations_sec": self.tts_durations_sec,
            "greeting_tts_duration_sec": self.greeting_tts_duration_sec,
            "num_turns": self.num_turns,
            "total_duration_sec": self.total_duration_sec,
            "end_reason": self.end_reason,
        }


class InteractionLogger:
    """Ring buffer of the last N voice interactions with JSON persistence."""

    def __init__(self, node: Node, log_path: str = DEFAULT_LOG_PATH):
        self._node = node
        self._log_path = log_path
        self._records: List[dict] = []

        # Publishers
        self._interaction_pub = node.create_publisher(
            VoiceInteraction, "/voice/interaction_log", 10
        )
        self._summary_pub = node.create_publisher(
            String, "/voice/interaction_summary", 10
        )

        # Load existing records from disk
        self._load()

    def start_interaction(
        self, wake_phrase: str, confidence: float
    ) -> InteractionRecord:
        """Begin tracking a new interaction. Returns a record to fill in."""
        record = InteractionRecord(
            start_time=time.monotonic(),
            start_wall=time.time(),
            wake_phrase=wake_phrase,
            wake_confidence=confidence,
        )
        return record

    def finish_interaction(self, record: InteractionRecord) -> None:
        """Finalize a record: append to buffer, publish, persist."""
        record.end_time = time.monotonic()
        record.end_wall = time.time()

        record_dict = record.to_dict()
        self._records.append(record_dict)

        # Cap to ring buffer size
        if len(self._records) > MAX_RECORDS:
            self._records = self._records[-MAX_RECORDS:]

        # Publish the individual interaction message
        self._publish_interaction(record)

        # Publish the full summary
        self._publish_summary()

        # Persist to disk
        self._save()

    def _publish_interaction(self, record: InteractionRecord) -> None:
        """Publish a VoiceInteraction message for this record."""
        msg = VoiceInteraction()

        msg.start_time = _wall_to_ros_time(record.start_wall)
        msg.end_time = _wall_to_ros_time(record.end_wall)

        msg.wake_phrase = record.wake_phrase
        msg.wake_confidence = record.wake_confidence

        msg.person_detected = record.person_detected
        msg.person_track_duration_sec = record.person_track_duration_sec

        msg.user_transcripts = record.user_transcripts
        msg.llm_responses = record.llm_responses
        msg.stt_durations_sec = record.stt_durations_sec
        msg.llm_durations_sec = record.llm_durations_sec
        msg.tts_durations_sec = record.tts_durations_sec

        msg.greeting_tts_duration_sec = record.greeting_tts_duration_sec

        msg.num_turns = record.num_turns
        msg.total_duration_sec = record.total_duration_sec
        msg.end_reason = record.end_reason

        self._interaction_pub.publish(msg)

    def _publish_summary(self) -> None:
        """Publish JSON summary of all records + aggregate stats."""
        msg = String()
        msg.data = self.get_summary_json()
        self._summary_pub.publish(msg)

    def get_summary_json(self) -> str:
        """Return JSON string with all records (newest first) and aggregate stats."""
        records_newest_first = list(reversed(self._records))
        stats = _compute_aggregate_stats(self._records)
        summary = {
            "interactions": records_newest_first,
            "count": len(self._records),
            "aggregate_stats": stats,
        }
        return json.dumps(summary, indent=2)

    def _load(self) -> None:
        """Load records from JSON file."""
        path = Path(self._log_path)
        if not path.exists():
            return
        try:
            data = json.loads(path.read_text())
            if isinstance(data, list):
                self._records = data[-MAX_RECORDS:]
            elif isinstance(data, dict) and "interactions" in data:
                # Handle summary format (interactions are newest-first)
                self._records = list(reversed(data["interactions"]))[-MAX_RECORDS:]
            self._node.get_logger().info(
                f"Loaded {len(self._records)} interaction records from {self._log_path}"
            )
        except (json.JSONDecodeError, OSError) as e:
            self._node.get_logger().warn(f"Could not load interaction log: {e}")

    def _save(self) -> None:
        """Persist records to JSON file."""
        path = Path(self._log_path)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(self._records, indent=2))
        except OSError as e:
            self._node.get_logger().warn(f"Could not save interaction log: {e}")


def _wall_to_ros_time(wall_secs: float) -> Time:
    """Convert a wall-clock float (time.time()) to a builtin_interfaces/Time."""
    t = Time()
    t.sec = int(wall_secs)
    t.nanosec = int((wall_secs - int(wall_secs)) * 1e9)
    return t


def _safe_avg(values: List[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def _compute_aggregate_stats(records: List[dict]) -> dict:
    """Compute aggregate timing stats across all records."""
    all_stt = []
    all_llm = []
    all_tts = []
    all_total = []
    all_turns = []

    for r in records:
        all_stt.extend(r.get("stt_durations_sec", []))
        all_llm.extend(r.get("llm_durations_sec", []))
        all_tts.extend(r.get("tts_durations_sec", []))
        total = r.get("total_duration_sec", 0.0)
        if total > 0:
            all_total.append(total)
        turns = r.get("num_turns", 0)
        all_turns.append(turns)

    def _stats(values: List[float]) -> dict:
        if not values:
            return {"avg": 0.0, "min": 0.0, "max": 0.0}
        return {
            "avg": round(_safe_avg(values), 3),
            "min": round(min(values), 3),
            "max": round(max(values), 3),
        }

    return {
        "stt": _stats(all_stt),
        "llm": _stats(all_llm),
        "tts": _stats(all_tts),
        "total_duration": _stats(all_total),
        "turns_per_interaction": _stats([float(t) for t in all_turns]),
    }
