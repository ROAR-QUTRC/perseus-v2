# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Best-effort JSON persistence of per-profile option selections (pure)."""

import json
import os
from pathlib import Path
from typing import Dict, Mapping, Optional

APP_ID = 'perseus_lite_tui'
FILENAME = 'selections.json'


def config_dir(env: Optional[Mapping[str, str]] = None) -> Path:
    """Return the XDG config directory for the TUI's state file."""
    env = os.environ if env is None else env
    base = env.get('XDG_CONFIG_HOME') or os.path.join(env.get('HOME', ''), '.config')
    return Path(base) / APP_ID


def state_path(env: Optional[Mapping[str, str]] = None) -> Path:
    """Return the full path to the selections state file."""
    return config_dir(env) / FILENAME


def load_selections(path: Path) -> Dict[str, Dict[str, str]]:
    """Load saved selections; return an empty mapping on any failure."""
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return {}
    if not isinstance(data, dict):
        return {}
    return data


def save_selections(path: Path, data: Mapping[str, Dict[str, str]]) -> bool:
    """Atomically write selections; never raise, return success as a bool."""
    path = Path(path)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name(path.name + '.tmp')
        with open(tmp, 'w', encoding='utf-8') as handle:
            json.dump(dict(data), handle, indent=2, sort_keys=True)
        os.replace(tmp, path)
        return True
    except OSError:
        return False
