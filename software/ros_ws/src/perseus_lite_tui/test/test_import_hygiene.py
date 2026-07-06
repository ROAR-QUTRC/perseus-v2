# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Assert the pure layer never pulls curses or rclpy into the process."""

import os
import subprocess
import sys

PURE_MODULES = (
    'perseus_lite_tui.config',
    'perseus_lite_tui.jobs',
    'perseus_lite_tui.layout',
    'perseus_lite_tui.persistence',
    'perseus_lite_tui.pixi_env',
    'perseus_lite_tui.registry',
    'perseus_lite_tui.topic_stats',
)


def test_pure_modules_do_not_import_curses_or_rclpy():
    package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    code = (
        'import sys\n'
        'import ' + ', '.join(PURE_MODULES) + '\n'
        "banned = {'curses', '_curses', 'rclpy'} & set(sys.modules)\n"
        "assert not banned, 'pure layer imported: %s' % sorted(banned)\n"
    )
    env = dict(os.environ)
    env['PYTHONPATH'] = package_root + os.pathsep + env.get('PYTHONPATH', '')
    result = subprocess.run(
        [sys.executable, '-c', code],
        capture_output=True,
        text=True,
        env=env,
    )
    assert result.returncode == 0, result.stderr
