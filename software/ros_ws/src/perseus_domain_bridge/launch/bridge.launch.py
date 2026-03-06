"""
domain_bridge.launch.py
=======================
Launch the standard (unthrottled) ros2/domain_bridge executable.

Usage
-----
ros2 launch domain_bridge_throttled domain_bridge.launch.py \\
    config:=/path/to/bridge.yaml \\
    from_domain:=0 \\
    to_domain:=1

Arguments
---------
config       Path to the domain_bridge YAML config file.
from_domain  Override all 'from_domain' values in the config (optional).
to_domain    Override all 'to_domain'   values in the config (optional).
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config",
        default_value="",
        description="Path to the domain_bridge YAML config file.",
    )
    from_domain_arg = DeclareLaunchArgument(
        "from_domain",
        default_value="",
        description="Override from_domain for all topics (optional).",
    )
    to_domain_arg = DeclareLaunchArgument(
        "to_domain",
        default_value="",
        description="Override to_domain for all topics (optional).",
    )

    def launch_setup(context, *args, **kwargs):
        config      = LaunchConfiguration("config").perform(context)
        from_domain = LaunchConfiguration("from_domain").perform(context)
        to_domain   = LaunchConfiguration("to_domain").perform(context)

        # Fall back to the example config shipped with domain_bridge itself
        if not config:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory("domain_bridge")
                config = os.path.join(pkg_share, "examples", "example_bridge_config.yaml")
            except Exception as exc:
                raise RuntimeError(
                    "domain_bridge package not found and no config:= provided."
                ) from exc

        cmd = ["ros2", "run", "domain_bridge", "domain_bridge"]
        if from_domain:
            cmd += ["--from", from_domain]
        if to_domain:
            cmd += ["--to", to_domain]
        cmd.append(config)

        return [
            ExecuteProcess(
                cmd=cmd,
                output="screen",
                name="domain_bridge",
            )
        ]

    return LaunchDescription([
        config_arg,
        from_domain_arg,
        to_domain_arg,
        OpaqueFunction(function=launch_setup),
    ])