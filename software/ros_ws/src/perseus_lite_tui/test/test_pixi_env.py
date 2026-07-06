# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Unit tests for repository/environment discovery helpers."""

from perseus_lite_tui import pixi_env
from perseus_lite_tui.registry import Profile


def test_find_repo_root(tmp_path):
    (tmp_path / 'pixi.toml').write_text('')
    nested = tmp_path / 'a' / 'b'
    nested.mkdir(parents=True)
    assert pixi_env.find_repo_root(nested) == tmp_path.resolve()


def test_find_repo_root_missing(tmp_path):
    nested = tmp_path / 'a'
    nested.mkdir()
    assert pixi_env.find_repo_root(nested) is None


def test_is_aarch64():
    assert pixi_env.is_aarch64('aarch64')
    assert pixi_env.is_aarch64('arm64')
    assert not pixi_env.is_aarch64('x86_64')


def test_visible_profiles_filters_linux64_on_arm():
    profiles = [
        Profile('a', 'A', 'pkg', 'a.launch.py'),
        Profile('b', 'B', 'pkg', 'b.launch.py', linux64_only=True),
    ]
    assert [p.slug for p in pixi_env.visible_profiles(profiles, 'aarch64')] == ['a']
    assert [p.slug for p in pixi_env.visible_profiles(profiles, 'x86_64')] == ['a', 'b']


def test_install_setup_exists(tmp_path):
    assert not pixi_env.install_setup_exists(tmp_path)
    setup = pixi_env.ros_ws_dir(tmp_path) / 'install' / 'setup.bash'
    setup.parent.mkdir(parents=True)
    setup.write_text('')
    assert pixi_env.install_setup_exists(tmp_path)
