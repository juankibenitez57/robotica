#!/bin/bash
# check_fsm.sh — Estado actual de la FSM cognitiva
echo "=== FSM STATE ==="
ros2 topic echo /cognitive/fsm_state --once 2>/dev/null || echo "  [no data]"
echo ""
echo "=== COGNITIVE STATUS ==="
ros2 topic echo /cognitive/status --once 2>/dev/null || echo "  [no data]"
echo ""
echo "=== NODO ACTIVO ==="
ros2 node list 2>/dev/null | grep -E "fsm|cognitive|nlp" || echo "  [ningún nodo cognitivo activo]"
