# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Pure text-layout helpers for the curses screens (no curses import)."""

from typing import List, Sequence, Tuple

ELLIPSIS = '…'


def clip(text: str, width: int) -> str:
    """Truncate `text` to `width` columns, adding an ellipsis when cut."""
    if width <= 0:
        return ''
    if len(text) <= width:
        return text
    if width == 1:
        return text[:1]
    return text[: width - 1] + ELLIPSIS


def pad(text: str, width: int) -> str:
    """Clip then right-pad `text` to exactly `width` columns."""
    return clip(text, width).ljust(width)


def clamp_scroll(offset: int, total: int, height: int) -> int:
    """Clamp a scroll offset so the last page stays fully visible."""
    max_offset = max(0, total - height)
    return max(0, min(offset, max_offset))


def visible_slice(
    lines: Sequence[str], offset: int, height: int
) -> Tuple[List[str], int]:
    """Return the visible window of `lines` and the clamped offset used."""
    if height <= 0:
        return [], 0
    offset = clamp_scroll(offset, len(lines), height)
    end = offset + height
    return list(lines[offset:end]), offset


def tail(lines: Sequence[str], height: int) -> List[str]:
    """Return the last `height` lines (for auto-scrolling logs)."""
    if height <= 0:
        return []
    return list(lines[-height:])


def tab_bar(titles: Sequence[str], active: int) -> List[Tuple[str, bool]]:
    """Return (label, is_active) segments for a numbered tab bar."""
    segments: List[Tuple[str, bool]] = []
    for index, title in enumerate(titles):
        segments.append((f' {index + 1}:{title} ', index == active))
    return segments
