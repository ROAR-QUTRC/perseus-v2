# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Unit tests for pure text-layout helpers."""

from perseus_lite_tui import layout


def test_clip_shorter_than_width():
    assert layout.clip('abc', 10) == 'abc'


def test_clip_truncates_with_ellipsis():
    result = layout.clip('abcdef', 4)
    assert len(result) == 4
    assert result.endswith(layout.ELLIPSIS)


def test_clip_zero_width():
    assert layout.clip('abc', 0) == ''


def test_pad_exact_width():
    assert layout.pad('ab', 5) == 'ab   '


def test_clamp_scroll():
    assert layout.clamp_scroll(100, 10, 5) == 5
    assert layout.clamp_scroll(-3, 10, 5) == 0
    assert layout.clamp_scroll(2, 10, 5) == 2


def test_visible_slice_clamps():
    lines = [str(i) for i in range(10)]
    window, offset = layout.visible_slice(lines, 100, 3)
    assert window == ['7', '8', '9']
    assert offset == 7


def test_tail():
    assert layout.tail(['a', 'b', 'c'], 2) == ['b', 'c']
    assert layout.tail(['a'], 5) == ['a']


def test_tab_bar_marks_active():
    segments = layout.tab_bar(['Launch', 'Logs'], 1)
    assert segments[0][1] is False
    assert segments[1][1] is True
    assert 'Launch' in segments[0][0]
