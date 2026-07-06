# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Rate and freshness math over monotonic arrival timestamps (pure)."""

from typing import Sequence

STALE_AFTER_S = 2.0


def rate_hz(timestamps: Sequence[float]) -> float:
    """Return the mean publish rate implied by a window of arrival times."""
    if len(timestamps) < 2:
        return 0.0
    span = timestamps[-1] - timestamps[0]
    if span <= 0.0:
        return 0.0
    return (len(timestamps) - 1) / span


def age_seconds(last_timestamp: float, now: float) -> float:
    """Return how long ago the last sample arrived, clamped at zero."""
    return max(0.0, now - last_timestamp)


def is_stale(age: float, threshold: float = STALE_AFTER_S) -> bool:
    """Return whether a topic is stale given the age of its last sample."""
    return age > threshold


def format_rate(rate: float) -> str:
    """Render a rate as a compact fixed-width string."""
    if rate <= 0.0:
        return '   -  '
    if rate >= 100.0:
        return f'{rate:5.0f}H'
    return f'{rate:5.1f}H'
