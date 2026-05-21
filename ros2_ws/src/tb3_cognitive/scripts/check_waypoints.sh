#!/bin/bash
# check_waypoints.sh — Valida waypoints.yaml
YAML="$(ros2 pkg prefix tb3_cognitive 2>/dev/null)/share/tb3_cognitive/config/waypoints.yaml"
[ ! -f "$YAML" ] && YAML="$(find ~/code/ROBOTICA -name waypoints.yaml 2>/dev/null | head -1)"

echo "=== WAYPOINTS FILE ==="
echo "  $YAML"
echo ""
if [ -f "$YAML" ]; then
    echo "=== CONTENIDO ==="
    cat "$YAML"
else
    echo "  [archivo no encontrado]"
fi
