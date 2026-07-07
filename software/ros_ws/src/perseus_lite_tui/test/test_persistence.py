# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Unit tests for selection persistence."""

from perseus_lite_tui import persistence


def test_config_dir_uses_xdg():
    env = {'XDG_CONFIG_HOME': '/xdg', 'HOME': '/home/u'}
    assert str(persistence.config_dir(env)) == '/xdg/perseus_lite_tui'


def test_config_dir_falls_back_to_home():
    env = {'HOME': '/home/u'}
    assert str(persistence.config_dir(env)) == '/home/u/.config/perseus_lite_tui'


def test_round_trip(tmp_path):
    path = tmp_path / 'selections.json'
    data = {'bringup': {'use_mock_hardware': 'True'}}
    assert persistence.save_selections(path, data) is True
    assert persistence.load_selections(path) == data


def test_load_missing_returns_empty(tmp_path):
    assert persistence.load_selections(tmp_path / 'nope.json') == {}


def test_load_corrupt_returns_empty(tmp_path):
    path = tmp_path / 'bad.json'
    path.write_text('{not valid json')
    assert persistence.load_selections(path) == {}


def test_save_creates_parent_dirs(tmp_path):
    path = tmp_path / 'nested' / 'dir' / 'selections.json'
    assert persistence.save_selections(path, {'a': {'b': 'c'}}) is True
    assert path.is_file()
