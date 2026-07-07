# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Validate that the profile/task config matches the workspace on disk."""

from pathlib import Path

from perseus_lite_tui import config, pixi_env


def _repo_root():
    root = pixi_env.find_repo_root(Path(__file__).resolve())
    assert root is not None, 'could not locate pixi.toml above the test file'
    return root


def test_profile_ids_are_unique():
    ids = [p.slug for p in config.PROFILES]
    assert len(ids) == len(set(ids))


def test_task_ids_are_unique():
    ids = [t.slug for t in config.TASKS]
    assert len(ids) == len(set(ids))


def test_every_profile_launch_file_exists():
    src = pixi_env.ros_ws_dir(_repo_root()) / 'src'
    missing = [
        str(src / p.package / 'launch' / p.launch_file)
        for p in config.PROFILES
        if not (src / p.package / 'launch' / p.launch_file).is_file()
    ]
    assert not missing, 'missing launch files: %s' % missing


def test_simulation_profiles_are_gated():
    sim = [p for p in config.PROFILES if p.env == 'simulation']
    assert sim, 'expected at least one simulation profile'
    assert all(p.linux64_only for p in sim)
