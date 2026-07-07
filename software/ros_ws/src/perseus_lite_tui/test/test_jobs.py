# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Unit tests for the subprocess job manager."""

import time

from perseus_lite_tui import jobs


def _wait_until(predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


def test_job_runs_and_captures_output():
    job = jobs.Job('echo', 'echo', ['bash', '-c', 'echo hello; echo world'])
    job.start()
    assert _wait_until(lambda: not job.running)
    log = job.snapshot_log()
    assert 'hello' in log
    assert 'world' in log
    assert job.returncode == 0
    assert job.status() == jobs.EXITED


def test_job_stop_terminates_process_group():
    job = jobs.Job('sleep', 'sleep', ['bash', '-c', 'sleep 30'])
    job.start()
    assert _wait_until(lambda: job.running)
    job.stop()
    assert _wait_until(lambda: not job.running)
    assert job.returncode is not None


def test_log_buffer_is_bounded():
    argv = ['bash', '-c', 'for i in $(seq 1 200); do echo line$i; done']
    job = jobs.Job('spam', 'spam', argv, log_max=10)
    job.start()
    assert _wait_until(lambda: not job.running)
    assert len(job.snapshot_log()) <= 10


def test_noise_filter_drops_lines():
    argv = ['bash', '-c', 'echo keep; echo NOISE; echo keep2']
    job = jobs.Job('nf', 'nf', argv, noise_filter=lambda line: line == 'NOISE')
    job.start()
    assert _wait_until(lambda: not job.running)
    assert 'NOISE' not in job.snapshot_log()
    assert 'keep' in job.snapshot_log()


def test_manager_one_job_per_key():
    manager = jobs.JobManager()
    job1, started1 = manager.start('a', 'A', ['bash', '-c', 'sleep 30'], None)
    assert started1
    job2, started2 = manager.start('a', 'A', ['bash', '-c', 'sleep 30'], None)
    assert not started2
    assert job1 is job2
    manager.stop_all()
    assert _wait_until(lambda: not manager.running_jobs())


def test_manager_clear_finished():
    manager = jobs.JobManager()
    job, _ = manager.start('done', 'done', ['bash', '-c', 'true'], None)
    assert _wait_until(lambda: not job.running)
    assert manager.clear_finished() == 1
    assert manager.get('done') is None
