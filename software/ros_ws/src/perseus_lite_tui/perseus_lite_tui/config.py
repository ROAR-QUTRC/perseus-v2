# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""perseus-lite launch profiles and developer tasks shown in the TUI."""

from perseus_lite_tui.registry import BOOL, Option, Profile, Task

_MOCK = Option('use_mock_hardware', 'Mock hardware', BOOL, 'False')

# Ordered as they appear in the Launch tab. Every launch_file is verified to
# exist on disk by test/test_config.py, so a rename upstream fails the suite.
PROFILES = (
    # --- Bringup -------------------------------------------------------------
    Profile(
        'bringup',
        'Robot bringup',
        'perseus_lite',
        'perseus_lite.launch.py',
        options=(_MOCK,),
        group='Bringup',
    ),
    Profile(
        'controllers',
        'Controllers only',
        'perseus_lite',
        'controllers.launch.py',
        options=(_MOCK,),
        group='Bringup',
    ),
    Profile(
        'rsp',
        'Robot state publisher',
        'perseus_lite',
        'robot_state_publisher.launch.py',
        group='Bringup',
    ),
    # --- Autonomy ------------------------------------------------------------
    Profile(
        'slam_nav2',
        'SLAM + Nav2',
        'perseus_lite',
        'perseus_lite_slam_and_nav2.launch.py',
        options=(_MOCK,),
        group='Autonomy',
    ),
    Profile(
        'nav2',
        'Nav2 only',
        'perseus_lite',
        'nav2.launch.py',
        group='Autonomy',
    ),
    Profile(
        'autonomy',
        'Autonomy stack',
        'autonomy',
        'autonomy.launch.py',
        group='Autonomy',
    ),
    Profile(
        'mapping',
        'SLAM Toolbox mapping',
        'autonomy',
        'mapping_using_slam_toolbox.launch.py',
        group='Autonomy',
    ),
    Profile(
        'handheld_mapping',
        'Handheld mapping',
        'autonomy',
        'handheld_mapping.launch.py',
        group='Autonomy',
    ),
    # --- Teleop --------------------------------------------------------------
    Profile(
        'input_controller',
        'Input router',
        'perseus_input',
        'controller.launch.py',
        group='Teleop',
    ),
    Profile(
        'xbox',
        'Xbox controller',
        'input_devices',
        'xbox_controller.launch.py',
        group='Teleop',
    ),
    Profile(
        'teleop_diagnostics',
        'Teleop diagnostics (TUI)',
        'teleop_diagnostics',
        'teleop_diagnostics.launch.py',
        group='Teleop',
        disabled=True,
        note='Nested full-screen TUI; run in a separate terminal, not as a job.',
    ),
    # --- Simulation (linux-64 only; needs the simulation Pixi env) -----------
    Profile(
        'sim',
        'Gazebo simulation',
        'perseus_lite_simulation',
        'perseus_sim.launch.py',
        env='simulation',
        group='Simulation',
        linux64_only=True,
        note='First launch may trigger a large simulation-env install.',
    ),
    Profile(
        'mission_zero',
        'Mission Zero (Selene)',
        'perseus_lite_missions',
        'mission_zero.launch.py',
        env='simulation',
        group='Simulation',
        linux64_only=True,
    ),
)

# Developer tasks. Each argv is a complete pixi invocation; the Pixi task itself
# carries the right cwd and package-skip flags.
TASKS = (
    Task('build', 'Build (default env)', ('pixi', 'run', '-e', 'default', 'build')),
    Task(
        'build_test',
        'Build with tests',
        ('pixi', 'run', '-e', 'default', 'build-test'),
    ),
    Task('test', 'Run unit tests', ('pixi', 'run', '-e', 'default', 'test')),
    Task('clean', 'Clean workspace', ('pixi', 'run', '-e', 'default', 'clean')),
    Task(
        'sim_build',
        'Build (simulation env)',
        ('pixi', 'run', '-e', 'simulation', 'build'),
        group='Build',
        note='linux-64 only.',
    ),
    Task('fmt', 'Format (treefmt)', ('pixi', 'run', '-e', 'format', 'fmt')),
)
