# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Live ROS graph monitor backed by a lazy rclpy daemon thread."""

# rclpy is imported lazily inside start()/the worker thread so the rest of the
# TUI runs (and imports) without ROS. If rclpy is missing or init fails the
# monitor self-disables and the Monitor tab explains why. Everything the render
# thread reads goes through snapshot() under a lock.

from collections import deque
import threading
import time
from typing import Dict, List, Optional

from perseus_lite_tui import topic_stats

# Curated topics shown first, in this order; everything else follows, sorted.
PINNED = (
    '/cmd_vel',
    '/odom',
    '/joint_states',
    '/scan',
    '/tf',
    '/map',
    '/diagnostics',
)

_DISCOVERY_INTERVAL_S = 2.0
_WINDOW = 100


class RosMonitor:
    """Subscribe to every graph topic and expose per-topic rate/type."""

    def __init__(self):
        self.available = False
        self.error = ''
        self._lock = threading.Lock()
        self._stamps: Dict[str, deque] = {}
        self._types: Dict[str, str] = {}
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._rclpy = None
        self._node = None
        self._context = None
        self._executor = None

    def start(self) -> None:
        """Import rclpy and spin the monitor node on a daemon thread."""
        try:
            import rclpy  # noqa: F401
        except Exception as exc:  # noqa: BLE001
            self.available = False
            self.error = f'rclpy unavailable: {exc}'
            return
        self._rclpy = rclpy
        self.available = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Signal the worker to shut down and wait briefly for it."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def snapshot(self) -> List[dict]:
        """Return per-topic rows sorted pinned-first, then alphabetically."""
        now = time.monotonic()
        with self._lock:
            items = [
                (name, self._types.get(name, ''), list(dq))
                for name, dq in self._stamps.items()
            ]
        rows = []
        for name, type_name, stamps in items:
            age = topic_stats.age_seconds(stamps[-1], now) if stamps else None
            rows.append(
                {
                    'topic': name,
                    'type': type_name,
                    'rate': topic_stats.rate_hz(stamps),
                    'age': age,
                }
            )
        order = {topic: index for index, topic in enumerate(PINNED)}
        rows.sort(key=lambda r: (order.get(r['topic'], len(PINNED)), r['topic']))
        return rows

    # -- worker thread --------------------------------------------------------

    def _run(self) -> None:
        rclpy = self._rclpy
        try:
            self._context = rclpy.Context()
            rclpy.init(context=self._context)
            self._node = rclpy.create_node('perseus_tui_monitor', context=self._context)
            self._executor = rclpy.executors.SingleThreadedExecutor(
                context=self._context
            )
            self._executor.add_node(self._node)
        except Exception as exc:  # noqa: BLE001
            self.available = False
            self.error = f'monitor init failed: {exc}'
            return
        subscribed: set = set()
        last_discovery = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now - last_discovery >= _DISCOVERY_INTERVAL_S:
                last_discovery = now
                self._discover(subscribed)
            self._executor.spin_once(timeout_sec=0.1)
        self._teardown()

    def _discover(self, subscribed: set) -> None:
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from rosidl_runtime_py.utilities import get_message

        try:
            topics = self._node.get_topic_names_and_types()
        except Exception:  # noqa: BLE001
            return
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for name, types in topics:
            if name in subscribed or not types:
                continue
            try:
                msg_type = get_message(types[0])
                self._node.create_subscription(
                    msg_type, name, self._make_callback(name), qos, raw=True
                )
            except Exception:  # noqa: BLE001
                continue
            subscribed.add(name)
            with self._lock:
                self._types[name] = types[0]
                self._stamps.setdefault(name, deque(maxlen=_WINDOW))

    def _make_callback(self, name: str):
        def _on_message(_msg) -> None:
            stamp = time.monotonic()
            with self._lock:
                buffer = self._stamps.get(name)
                if buffer is not None:
                    buffer.append(stamp)

        return _on_message

    def _teardown(self) -> None:
        try:
            if self._executor is not None:
                self._executor.shutdown()
            if self._node is not None:
                self._node.destroy_node()
            if self._context is not None:
                self._rclpy.shutdown(context=self._context)
        except Exception:  # noqa: BLE001
            pass
