"""
test_bridge.launch.py
================================
Integration test that launches all three components in one go:

  1. test_talker   – publishes std_msgs/String on domain 0 at 60 Hz
  2. throttled_bridge – bridges /chatter from domain 0 → domain 1 at 30 Hz
  3. test_listener – subscribes on domain 1 and reports received Hz

Watch the test_listener console output.  You should see ~30 msg/s arriving
even though the talker is publishing at 60 Hz.

Usage
-----
ros2 launch perseus_domain_brdige test_throttled_bridge.launch.py

Optional overrides:
  talker_hz:=60        Source publish rate  (default 60)
  bridge_hz:=30        Bridge throttle rate (default 30)
  from_domain:=0       Source DDS domain    (default 0)
  to_domain:=1         Dest   DDS domain    (default 1)
  topic:=/chatter      Topic name           (default /chatter)
"""

import os
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("talker_hz",   default_value="60",       description="Talker publish rate (Hz)"),
        DeclareLaunchArgument("bridge_hz",   default_value="30",       description="Bridge throttle rate (Hz)"),
        DeclareLaunchArgument("from_domain", default_value="0",        description="Source DDS domain ID"),
        DeclareLaunchArgument("to_domain",   default_value="1",        description="Destination DDS domain ID"),
        DeclareLaunchArgument("topic",       default_value="/chatter", description="Topic name"),
    ]

    def launch_setup(context, *args, **kwargs):
        talker_hz   = LaunchConfiguration("talker_hz").perform(context)
        bridge_hz   = LaunchConfiguration("bridge_hz").perform(context)
        from_domain = LaunchConfiguration("from_domain").perform(context)
        to_domain   = LaunchConfiguration("to_domain").perform(context)
        topic       = LaunchConfiguration("topic").perform(context)

        # ── Write a temporary bridge config from the launch args ──────────
        config_content = f"""\
from_domain: {from_domain}
to_domain: {to_domain}

topics:
  {topic}:
    type: std_msgs/msg/String
    max_rate: {bridge_hz}
    qos: best_effort
"""
        tmp_cfg = tempfile.NamedTemporaryFile(
            mode="w", suffix="_test_bridge.yaml", delete=False
        )
        tmp_cfg.write(config_content)
        tmp_cfg.flush()
        tmp_cfg.close()
        cfg_path = tmp_cfg.name

        intro = LogInfo(
            msg=(
                f"\n"
                f"  ┌─────────────────────────────────────────────┐\n"
                f"  │  domain_bridge_throttled – Bridge Test       │\n"
                f"  ├─────────────────────────────────────────────┤\n"
                f"  │  talker  : domain {from_domain} @ {talker_hz} Hz on {topic:<16}│\n"
                f"  │  bridge  : domain {from_domain} → {to_domain}  throttled to {bridge_hz} Hz       │\n"
                f"  │  listener: domain {to_domain}  (watch for ~{bridge_hz} msg/s)      │\n"
                f"  └─────────────────────────────────────────────┘\n"
            )
        )

        # ── 1. Talker on from_domain ───────────────────────────────────────
        talker = Node(
            package="perseus_domain_bridge",
            executable="test_talker",
            name="test_talker",
            output="screen",
            arguments=[from_domain, topic, talker_hz],
        )

        # ── 2. Throttled bridge ───────────────────────────────────────────
        # Small delay so the talker is publishing before the bridge starts
        bridge = TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="perseus_domain_bridge",
                    executable="bridge",
                    name="bridge",
                    output="screen",
                    arguments=[cfg_path],
                )
            ],
        )

        # ── 3. Listener on to_domain ──────────────────────────────────────
        # Another short delay so the bridge is up before the listener starts
        listener = TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="perseus_domain_bridge",
                    executable="test_listener",
                    name="test_listener",
                    output="screen",
                    arguments=[to_domain, topic],
                )
            ],
        )

        return [intro, talker, bridge, listener]

    return LaunchDescription([*args, OpaqueFunction(function=launch_setup)])