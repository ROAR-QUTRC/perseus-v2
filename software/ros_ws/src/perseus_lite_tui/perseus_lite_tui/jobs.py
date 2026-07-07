# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Subprocess job management: process groups, log ring, signal escalation."""

# Touches the OS (subprocess/signal/threading) but imports neither curses nor
# rclpy, so it stays inside the pure-layer import fence and is directly
# unit-testable with short-lived shell commands.

import collections
import os
import signal
from subprocess import PIPE, Popen, STDOUT
import threading
import time
from typing import Callable, Dict, List, Optional

RUNNING = 'running'
STOPPING = 'stopping'
EXITED = 'exited'

# Graceful-stop escalation windows.
_SIGTERM_AFTER_S = 10.0
_SIGKILL_AFTER_S = 3.0
_POLL_INTERVAL_S = 0.1


class Job:
    """A single launched command with a bounded, thread-safe log buffer."""

    def __init__(
        self,
        key: str,
        label: str,
        argv: List[str],
        cwd: Optional[str] = None,
        log_max: int = 4000,
        noise_filter: Optional[Callable[[str], bool]] = None,
    ):
        self.key = key
        self.label = label
        self.argv = list(argv)
        self.cwd = cwd
        self._log = collections.deque(maxlen=log_max)
        self._lock = threading.Lock()
        self._proc: Optional[Popen] = None
        self._stopping = False
        self._started_at: Optional[float] = None
        self._noise_filter = noise_filter

    def start(self) -> None:
        """Spawn the process in its own session and begin draining output."""
        if self._proc is not None:
            return
        self._started_at = time.monotonic()
        self._proc = Popen(
            self.argv,
            cwd=self.cwd,
            stdout=PIPE,
            stderr=STDOUT,
            start_new_session=True,
            text=True,
            errors='replace',
            bufsize=1,
        )
        reader = threading.Thread(target=self._drain, daemon=True)
        reader.start()

    def _drain(self) -> None:
        stream = self._proc.stdout if self._proc else None
        if stream is None:
            return
        for line in stream:
            text = line.rstrip('\n')
            if self._noise_filter is not None and self._noise_filter(text):
                continue
            with self._lock:
                self._log.append(text)
        try:
            stream.close()
        except OSError:
            pass

    def snapshot_log(self) -> List[str]:
        """Return a copy of the current log buffer."""
        with self._lock:
            return list(self._log)

    @property
    def running(self) -> bool:
        """Return whether the process is still alive."""
        return self._proc is not None and self._proc.poll() is None

    @property
    def returncode(self) -> Optional[int]:
        """Return the process exit code, or None if unstarted/still running."""
        return None if self._proc is None else self._proc.poll()

    def status(self) -> str:
        """Return one of RUNNING / STOPPING / EXITED."""
        if self._proc is None or self._proc.poll() is not None:
            return EXITED
        return STOPPING if self._stopping else RUNNING

    def status_line(self) -> str:
        """Return a short human-readable status string."""
        if self._proc is None:
            return 'not started'
        rc = self._proc.poll()
        if rc is None:
            elapsed = time.monotonic() - (self._started_at or time.monotonic())
            state = 'stopping' if self._stopping else 'running'
            return f'{state} {elapsed:.0f}s'
        return f'exited ({rc})'

    def stop(self) -> None:
        """Send SIGINT to the process group, escalating in the background."""
        if self._proc is None or self._proc.poll() is not None:
            return
        self._stopping = True
        try:
            pgid = os.getpgid(self._proc.pid)
        except ProcessLookupError:
            return
        self._signal_group(pgid, signal.SIGINT)
        escalator = threading.Thread(target=self._escalate, args=(pgid,), daemon=True)
        escalator.start()

    def _escalate(self, pgid: int) -> None:
        if self._wait_exit(_SIGTERM_AFTER_S):
            return
        self._signal_group(pgid, signal.SIGTERM)
        if self._wait_exit(_SIGKILL_AFTER_S):
            return
        self._signal_group(pgid, signal.SIGKILL)

    def _wait_exit(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._proc is None or self._proc.poll() is not None:
                return True
            time.sleep(_POLL_INTERVAL_S)
        return self._proc is None or self._proc.poll() is not None

    def _signal_group(self, pgid: int, sig: int) -> None:
        try:
            os.killpg(pgid, sig)
        except (ProcessLookupError, PermissionError):
            pass


class JobManager:
    """Owns the set of jobs; at most one live job per key."""

    def __init__(self, noise_filter: Optional[Callable[[str], bool]] = None):
        self._jobs: Dict[str, Job] = {}
        self._order: List[str] = []
        self._noise_filter = noise_filter

    def start(self, key: str, label: str, argv: List[str], cwd: Optional[str]):
        """Start a job for `key`; a no-op (returns started=False) if live."""
        existing = self._jobs.get(key)
        if existing is not None and existing.running:
            return existing, False
        job = Job(key, label, argv, cwd, noise_filter=self._noise_filter)
        job.start()
        self._jobs[key] = job
        if key not in self._order:
            self._order.append(key)
        return job, True

    def get(self, key: str) -> Optional[Job]:
        """Return the job for `key`, if any."""
        return self._jobs.get(key)

    def jobs(self) -> List[Job]:
        """Return jobs in start order."""
        return [self._jobs[k] for k in self._order if k in self._jobs]

    def running_jobs(self) -> List[Job]:
        """Return only the jobs that are still alive."""
        return [job for job in self.jobs() if job.running]

    def stop(self, key: str) -> None:
        """Stop the job for `key`, if any."""
        job = self._jobs.get(key)
        if job is not None:
            job.stop()

    def stop_all(self) -> None:
        """Stop every managed job (called on exit)."""
        for job in self.jobs():
            job.stop()

    def clear_finished(self) -> int:
        """Forget jobs that have exited; return how many were removed."""
        finished = [k for k in self._order if not self._jobs[k].running]
        for key in finished:
            del self._jobs[key]
            self._order.remove(key)
        return len(finished)
