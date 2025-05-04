#! /usr/bin/env bash
sleep 2.0
ros2 topic pub /key_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" -r 10 -t 10
ros2 topic pub /key_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -3.0}}}" -r 10 -t 5
ros2 topic pub /key_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" -r 10 -t 20
ros2 topic pub /key_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" -r 10 -1
