# ROBOTICA — Contexto del proyecto para Claude Code

## Entorno
- **OS**: Ubuntu 24.04 en WSL2 (WSLg para GUI)
- **ROS2**: Jazzy
- **Simulador**: Gazebo Harmonic
- **Robot**: TurtleBot3 Waffle
- **Workspace**: `~/code/ROBOTICA/ros2_ws`
- **Python IA**: `/opt/ai-venv` (sentence-transformers, LangChain, etc.) — separado del Python de sistema

## Reglas CRÍTICAS (nunca violar)
- **NUNCA romper NAV2, SLAM, Gazebo, TF ni RViz**
- `base_footprint` es el robot_base_frame (NO `base_link`)
- Cadena TF: `map → odom → base_footprint → base_link → base_scan`
- Los paquetes del curso (`diff_robot_urdf`, `image_folder_publisher`, `sentiment_ros2`) **nunca se tocan**
- PYTHONPATH debe **preponerse**, nunca sobreescribirse: `export PYTHONPATH="/opt/ai-venv/lib/python3.12/site-packages:${PYTHONPATH:-}"`
- Nodos IA usan `prefix='/opt/ai-venv/bin/python3'` en el `Node()` del launcher

## Arquitectura cognitiva del proyecto

```
Texto natural
    ↓
[nlp_intent_node]  ← sentence-transformers multilingual
    ↓  /cognitive/command  (tb3_msgs/CognitiveCommand)
[cognitive_fsm_node]  ← FSM: IDLE/NAVIGATING/EXPLORING/SEARCHING/ERROR
    ↓  NavigateToPose action
[NAV2]
    ↓
Robot se mueve en Gazebo
```

Topics clave:
- `/nlp/input` (std_msgs/String) — entrada de texto
- `/cognitive/command` (tb3_msgs/CognitiveCommand) — acción + target + confianza
- `/cognitive/fsm_state` (std_msgs/String) — estado FSM actual
- `/cognitive/status` (std_msgs/String) — resultado legible

## Estado de las fases

| Fase | Descripción | Estado |
|------|-------------|--------|
| 1 | NLP — texto → intención (sentence similarity multilingüe) | ✅ Completa |
| 2 | LangChain agent → NAV2 (fire-and-forget) | ✅ Completa |
| 3 | FSM cognitiva con goal tracking real | ✅ Completa |
| 4 | YOLOv8 percepción visual con cámara RGB | 🔜 Pendiente |
| 5 | Navegación cognitiva completa (misiones dinámicas) | 🔜 Pendiente |

## Paquetes del proyecto

### tb3_msgs (ament_cmake)
Mensaje custom: `CognitiveCommand.msg`
```
std_msgs/Header header
string action      # navigate | explore | search | approach | stop | report
string target      # cocina | botella | persona | ...
float32 confidence
string raw_text
```

### tb3_cognitive (ament_python)
```
tb3_cognitive/
  nlp/intent_node.py          — NLP con sentence-transformers
  langchain_agent/agent_node.py — Fase 2 (fire-and-forget, se mantiene)
  langchain_agent/tools.py
  fsm/states.py               — RobotFSM + enum State + tabla de transiciones
  fsm/fsm_node.py             — CognitiveFSMNode (Fase 3 activa)
scripts/
  nlp_intent_node             — wrapper con shebang /opt/ai-venv
  cognitive_agent_node        — wrapper Fase 2
  fsm_node                    — wrapper Fase 3
launch/
  cognitive_agent.launch.py   — Fase 2
  cognitive_fsm.launch.py     — Fase 3 (usar éste)
config/waypoints.yaml         — coordenadas de habitaciones en frame map
```

### tb3_bringup (ament_python)
- `launch/navigation.launch.py` — NAV2 completo (AMCL + planners + RViz)
- `launch/mapping.launch.py` — SLAM Toolbox
- `maps/house_map.yaml` — mapa guardado del entorno

## Comandos de uso

### Lanzar el sistema completo (Fase 3)
```bash
# Terminal 1 — NAV2
ros2 launch tb3_bringup navigation.launch.py

# Terminal 2 — Capa cognitiva FSM
ros2 launch tb3_cognitive cognitive_fsm.launch.py

# Terminal 3 — Monitorear estado
ros2 topic echo /cognitive/fsm_state

# Terminal 4 — Enviar comando
ros2 topic pub --once /nlp/input std_msgs/String "data: 've a la cocina'"
```

### Compilar
```bash
cd ~/code/ROBOTICA/ros2_ws && source install/setup.bash
colcon build --packages-select tb3_cognitive tb3_msgs
```

### Calibrar waypoints
1. Lanzar navigation.launch.py
2. En RViz: botón "Publish Point" → click en el mapa
3. `ros2 topic echo /clicked_point`
4. Pegar coordenadas en `config/waypoints.yaml`

## Decisiones técnicas importantes

- **`setup.cfg`** en tb3_cognitive es CRÍTICO: instala scripts en `lib/tb3_cognitive/` (no en `bin/`), que es donde `ros2 run` busca ejecutables
- **pip reescribe shebangs**: aunque el script tenga `#!/opt/ai-venv/bin/python3`, pip lo sobreescribe. La solución es `prefix='/opt/ai-venv/bin/python3'` en el `Node()` del launcher
- **numpy ABI**: sklearn del sistema (apt) es incompatible con numpy 2.x del ai-venv. Solución: `sudo /opt/ai-venv/bin/pip install "scikit-learn>=1.4"`
- **Modelo NLP**: `sentence-transformers/paraphrase-multilingual-MiniLM-L12-v2` — 50+ idiomas, clasificación por similitud de centroide (93% precisión en español/inglés)
- **conda**: desactivar auto_activate_base: `conda config --set auto_activate_base false`

## Procedimiento SLAM Mapping
```bash
# Terminal 1
ros2 launch tb3_bringup sim.launch.py

# Terminal 2
ros2 launch tb3_bringup mapping.launch.py

# Terminal 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4 — guardar mapa cuando esté completo
ros2 run nav2_map_server map_saver_cli -f ~/code/ROBOTICA/ros2_ws/src/tb3_bringup/maps/nombre_mapa
```
Parámetro crítico en mapper_params: `use_sim_time: true`
