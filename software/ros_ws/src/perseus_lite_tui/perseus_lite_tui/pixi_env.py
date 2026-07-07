# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Repository/environment discovery helpers (pure: no rclpy, no curses)."""

from pathlib import Path
import platform
from typing import List, Optional, Sequence

from perseus_lite_tui.registry import Profile


def find_repo_root(start: Path) -> Optional[Path]:
    """Walk upward from `start` and return the dir containing pixi.toml."""
    start = Path(start).resolve()
    candidates = [start, *start.parents] if start.is_dir() else list(start.parents)
    for directory in candidates:
        if (directory / 'pixi.toml').is_file():
            return directory
    return None


def ros_ws_dir(repo_root: Path) -> Path:
    """Return the colcon workspace directory for a repo root."""
    return Path(repo_root) / 'software' / 'ros_ws'


def install_setup_exists(repo_root: Path) -> bool:
    """Return whether the workspace overlay has been built (setup.bash present)."""
    return (ros_ws_dir(repo_root) / 'install' / 'setup.bash').is_file()


def is_aarch64(machine: Optional[str] = None) -> bool:
    """Return whether the given (or current) machine is 64-bit ARM."""
    value = machine if machine is not None else platform.machine()
    return value.lower() in ('aarch64', 'arm64')


def visible_profiles(
    profiles: Sequence[Profile], machine: Optional[str] = None
) -> List[Profile]:
    """Drop linux-64-only profiles when running on an aarch64 machine."""
    if not is_aarch64(machine):
        return list(profiles)
    return [p for p in profiles if not p.linux64_only]
