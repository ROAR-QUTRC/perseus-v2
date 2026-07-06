# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Declarative launch/task registry and command building (pure, no I/O)."""

from dataclasses import dataclass
import shlex
from typing import Dict, List, Tuple

# Option kinds.
BOOL = 'bool'
CHOICE = 'choice'
STRING = 'str'

_TRUE_WORDS = frozenset({'1', 'true', 'yes', 'on'})


@dataclass(frozen=True)
class Option:
    """A single launch argument the user can edit before launching."""

    key: str
    label: str
    kind: str = BOOL
    default: str = 'False'
    choices: Tuple[str, ...] = ()


@dataclass(frozen=True)
class Profile:
    """A launchable ROS entry point plus the Pixi env it must run in."""

    slug: str
    label: str
    package: str
    launch_file: str
    options: Tuple[Option, ...] = ()
    env: str = 'default'
    group: str = 'Bringup'
    linux64_only: bool = False
    disabled: bool = False
    note: str = ''


@dataclass(frozen=True)
class Task:
    """A one-shot developer command (already a full pixi invocation)."""

    slug: str
    label: str
    argv: Tuple[str, ...]
    group: str = 'Build'
    note: str = ''


def normalize_bool(value: str) -> str:
    """Return the canonical "True"/"False" form of a boolean-ish value."""
    return 'True' if str(value).strip().lower() in _TRUE_WORDS else 'False'


def canonical_value(option: Option, value: str) -> str:
    """Return the value normalized to its option kind."""
    if option.kind == BOOL:
        return normalize_bool(value)
    return str(value)


def option_args(profile: Profile, selections: Dict[str, str]) -> List[str]:
    """Return `key:=value` args only for options changed from their default."""
    # Emitting only changed options keeps a plain launch identical to the
    # hand-written pixi `bringup` task, so behaviour is easy to reason about.
    args: List[str] = []
    for option in profile.options:
        raw = selections.get(option.key, option.default)
        value = canonical_value(option, raw)
        if value != canonical_value(option, option.default):
            args.append(f'{option.key}:={value}')
    return args


def build_launch_command(
    profile: Profile, selections: Dict[str, str], ros_ws: str
) -> List[str]:
    """Build the argv that launches `profile` in its Pixi environment."""
    # Routed through `pixi run -e <env>` so the env activation (incl. the
    # path-sanitize hook) runs first and simulation profiles work from the
    # default-env TUI. `bash -c` (not `-lc`) avoids re-sourcing ~/.bashrc and
    # re-contaminating the environment after the sanitize hook ran.
    setup = shlex.quote(f'{ros_ws}/install/setup.bash')
    launch = ' '.join(
        [
            'ros2',
            'launch',
            profile.package,
            profile.launch_file,
            *option_args(profile, selections),
        ]
    )
    inner = f'source {setup} && {launch}'
    return ['pixi', 'run', '-e', profile.env, '--', 'bash', '-c', inner]


def build_task_command(task: Task) -> List[str]:
    """Return the argv for a one-shot developer task."""
    return list(task.argv)
