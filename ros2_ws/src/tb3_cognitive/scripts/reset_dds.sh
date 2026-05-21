#!/bin/bash
# reset_dds.sh — Limpia procesos DDS y ROS2 zombis
echo "=== Matando procesos ROS2/Gazebo zombis ==="
pkill -f "ros2"       2>/dev/null && echo "  ros2 OK" || echo "  ros2: ninguno"
pkill -f "gz sim"     2>/dev/null && echo "  gz OK"   || echo "  gz: ninguno"
pkill -f "gzserver"   2>/dev/null
pkill -f "gzclient"   2>/dev/null
pkill -f "rviz2"      2>/dev/null && echo "  rviz2 OK" || echo "  rviz2: ninguno"
pkill -f "bt_navigator\|planner_server\|controller_server\|costmap\|amcl" 2>/dev/null

sleep 1

echo ""
echo "=== Limpiando directorio ROS_DOMAIN_ID shared memory ==="
rm -f /tmp/fastrtps_* 2>/dev/null
rm -f /dev/shm/fastrtps_* 2>/dev/null && echo "  shm limpiado" || echo "  shm: nada que limpiar"

echo ""
echo "=== Procesos ROS2 restantes ==="
pgrep -la "ros2\|gz\|rviz" 2>/dev/null || echo "  [ninguno]"

echo ""
echo "Listo. Espera 2s antes de relanzar."
sleep 2
