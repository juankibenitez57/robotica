#!/bin/bash
# check_topics.sh — Todos los topics cognitivos activos
echo "=== TOPICS COGNITIVOS ==="
ros2 topic list 2>/dev/null | grep -E "cognitive|nlp|cmd_vel" || echo "  [sin topics cognitivos]"
echo ""
echo "=== HZ /cognitive/command ==="
timeout 3 ros2 topic hz /cognitive/command 2>/dev/null | head -3 || echo "  [sin publicaciones]"
echo ""
echo "=== ÚLTIMO /cognitive/command ==="
ros2 topic echo /cognitive/command --once 2>/dev/null || echo "  [sin mensajes]"
