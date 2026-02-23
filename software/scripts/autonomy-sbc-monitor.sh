#!/usr/bin/env bash

set -euo pipefail

echo "=== SBC Topic Isolation Monitor ==="
echo ""

# Check CycloneDDS configuration
echo "--- CycloneDDS Configuration ---"
if [ -n "${CYCLONEDDS_URI:-}" ]; then
  echo "CYCLONEDDS_URI: $CYCLONEDDS_URI"
else
  echo "WARNING: CYCLONEDDS_URI is not set!"
  echo "  Run: source ~/.config/cyclonedds/env.sh"
fi
echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-not set}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-not set (default 0)}"
echo ""

# Show network interfaces
echo "--- Network Interfaces ---"
ip -brief link show 2>/dev/null || echo "ip command not available"
echo ""

# Check for active ROS topics
echo "--- Active ROS Topics ---"
if command -v ros2 &>/dev/null; then
  ros2 topic list -v 2>/dev/null || echo "Failed to list topics (is a ROS node running?)"
  echo ""

  # Check specifically for pointcloud topics that should NOT be on the network
  echo "--- Pointcloud Topics (should be localhost-only) ---"
  POINTCLOUD_TOPICS=$(ros2 topic list 2>/dev/null | grep -i -E "livox|pointcloud|points" || true)
  if [ -n "$POINTCLOUD_TOPICS" ]; then
    echo "Found pointcloud topics (these should only be visible locally):"
    echo "$POINTCLOUD_TOPICS"
    echo ""
    echo "Checking bandwidth:"
    for topic in $POINTCLOUD_TOPICS; do
      echo "  $topic:"
      # Measure bandwidth for 3 seconds
      timeout 4 ros2 topic bw "$topic" 2>/dev/null | tail -1 || echo "    (no data or timeout)"
    done
  else
    echo "No pointcloud topics found (expected if not running on SBC)"
  fi
  echo ""

  # Show bridged topics
  echo "--- Bridge Status ---"
  BRIDGE_TOPICS="/cmd_vel /tf /tf_static /map /odom /scan /goal_pose /cmd_vel_joy"
  for topic in $BRIDGE_TOPICS; do
    COUNT=$(ros2 topic info "$topic" 2>/dev/null | grep -c "Publisher" || true)
    if [ "$COUNT" -gt 0 ]; then
      echo "  $topic: active"
    else
      echo "  $topic: not found"
    fi
  done
else
  echo "ros2 command not found. Run this script within the dev shell."
fi
