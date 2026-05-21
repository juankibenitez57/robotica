#!/bin/bash
# check_nav2.sh — Estado del stack NAV2
echo "=== NODOS NAV2 ACTIVOS ==="
ros2 node list 2>/dev/null | grep -E "bt_navigator|planner|controller|costmap|amcl|map" || echo "  [NAV2 no activo]"
echo ""
echo "=== ACTION SERVER navigate_to_pose ==="
ros2 action list 2>/dev/null | grep navigate_to_pose || echo "  [no disponible]"
echo ""
echo "=== TOPICS NAV2 ==="
ros2 topic list 2>/dev/null | grep -E "^/map$|^/odom$|^/scan$|navigate|costmap" || echo "  [sin topics NAV2]"
echo ""
echo "=== TF MAP→ODOM ==="
ros2 run tf2_ros tf2_echo map odom --timeout 2.0 2>&1 | head -8 || echo "  [TF no disponible]"
