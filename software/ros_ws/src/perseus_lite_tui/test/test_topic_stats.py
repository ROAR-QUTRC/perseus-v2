# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Unit tests for rate/freshness math."""

from perseus_lite_tui import topic_stats


def test_rate_hz_needs_two_samples():
    assert topic_stats.rate_hz([]) == 0.0
    assert topic_stats.rate_hz([1.0]) == 0.0


def test_rate_hz_ten_hz():
    stamps = [i * 0.1 for i in range(11)]  # 10 intervals of 0.1 s over 1.0 s
    assert abs(topic_stats.rate_hz(stamps) - 10.0) < 1e-9


def test_rate_hz_zero_span():
    assert topic_stats.rate_hz([5.0, 5.0]) == 0.0


def test_age_seconds_clamped():
    assert topic_stats.age_seconds(10.0, 12.5) == 2.5
    assert topic_stats.age_seconds(10.0, 9.0) == 0.0


def test_is_stale():
    assert topic_stats.is_stale(3.0)
    assert not topic_stats.is_stale(0.5)


def test_format_rate():
    assert topic_stats.format_rate(0.0).strip() == '-'
    assert 'H' in topic_stats.format_rate(12.3)
