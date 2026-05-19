# ROBOTICA — ROS2 Jazzy + Gazebo Harmonic

Proyecto de robótica con **TurtleBot3 Waffle** en simulación.  
Stack completo: simulación Gazebo → SLAM → mapa → navegación autónoma NAV2.

---

## Requisitos previos

- Ubuntu 24.04 (WSL2 o nativo)
- ROS2 Jazzy instalado
- Gazebo Harmonic (gz-sim 8)
- Paquetes del curso instalados (ver `README_instalacion_wsl_ubuntu2404_jazzy.md`)

Comprueba que tienes todo con:
```bash
ros2 pkg list | grep -E "slam_toolbox|nav2_bringup|ros_gz_bridge|tb3"
```

---

## Clonar y compilar

```bash
cd ~
git clone https://github.com/juankibenitez57/robotica.git code/ROBOTICA
cd code/ROBOTICA/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Añade el source al `.bashrc` para no repetirlo:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/code/ROBOTICA/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Estructura del proyecto

```
ros2_ws/src/
├── tb3_description/          # URDF/SDF del TurtleBot3 Waffle
│   └── urdf/
│       ├── gz_waffle.sdf.xacro   # modelo Gazebo (gpu_lidar, DiffDrive, IMU)
│       └── tb3_waffle.urdf       # modelo ROS (robot_state_publisher)
│
├── tb3_bringup/              # Launch files y configuración
│   ├── launch/
│   │   ├── bringup.launch.py     # LAUNCHER PRINCIPAL ← usa este
│   │   ├── gazebo.launch.py      # Gazebo + bridge + RSP
│   │   ├── slam.launch.py        # SLAM Toolbox (mapeo)
│   │   ├── localization.launch.py # AMCL (localización con mapa)
│   │   └── navigation.launch.py  # NAV2 (planificador + controlador)
│   ├── config/
│   │   ├── bridge.yaml           # topics Gazebo ↔ ROS2
│   │   ├── mapper_params_online_async.yaml  # parámetros SLAM
│   │   ├── nav2_params.yaml      # parámetros NAV2 completo
│   │   └── rviz/
│   │       ├── slam_view.rviz    # RViz para mapeo
│   │       └── nav2.rviz         # RViz para navegación
│   ├── worlds/
│   │   └── tb3_lab.sdf           # mundo Gazebo (campo de fútbol)
│   └── maps/
│       └── house_map.*           # mapa guardado (pgm + yaml)
│
├── diff_robot_urdf/          # CP1 — robot diferencial básico
├── image_folder_publisher/   # CP1 — publisher de imágenes
└── sentiment_ros2/           # CP2 — análisis de sentimiento
```

---

## Modo 1: SLAM — Crear un mapa nuevo

Sirve para explorar el entorno y construir el mapa desde cero.

### Terminal 1 — Lanzar todo
```bash
ros2 launch tb3_bringup bringup.launch.py nav_mode:=slam
```

Espera ~30 segundos hasta que Gazebo cargue completamente.  
RViz abrirá con vista TopDown. Al inicio verás "Fixed Frame [map] does not exist" — **es normal**, desaparece cuando SLAM recibe el primer scan.

### Terminal 2 — Mover el robot
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

| Tecla | Acción |
|-------|--------|
| `i` | Avanzar |
| `,` | Retroceder |
| `j` | Girar izquierda |
| `l` | Girar derecha |
| `k` | Parar |
| `q/z` | +/- velocidad lineal |
| `w/x` | +/- velocidad angular |

Mueve el robot por todo el entorno. El mapa se construye en tiempo real en RViz.

### Terminal 3 — Guardar el mapa cuando termines
```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/code/ROBOTICA/ros2_ws/src/tb3_bringup/maps/mi_mapa
```
Crea dos archivos: `mi_mapa.pgm` (imagen) y `mi_mapa.yaml` (metadatos).

---

## Modo 2: Navegación autónoma — Usar mapa guardado

Con el mapa guardado, el robot se localiza (AMCL) y navega solo a objetivos.

### Terminal 1 — Lanzar todo con el mapa
```bash
ros2 launch tb3_bringup bringup.launch.py nav_mode:=navigation \
  map:=~/code/ROBOTICA/ros2_ws/src/tb3_bringup/maps/house_map.yaml
```

### Enviar objetivo desde RViz
1. En RViz, click en el botón **"2D Goal Pose"** (barra superior)
2. Click en el mapa donde quieres que vaya el robot
3. El robot planifica la ruta y navega solo evitando obstáculos

### Enviar objetivo por terminal
```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{ header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5, z: 0.0},
     orientation: {w: 1.0}}}" --once
```

---

## Arquitectura técnica

```
┌─────────────────────────────────────────────────────────────┐
│                        GAZEBO HARMONIC                       │
│                                                              │
│  TurtleBot3 Waffle SDF                                       │
│  ├── gpu_lidar sensor  → topic gz /scan (5 Hz)              │
│  ├── IMU sensor        → topic gz /imu  (200 Hz)            │
│  └── DiffDrive plugin  → topics gz /odom + /tf + /cmd_vel   │
└──────────────────────┬──────────────────────────────────────┘
                       │  ros_gz_bridge (parameter_bridge)
                       │  bridge.yaml — traduce tipos gz ↔ ROS2
┌──────────────────────▼──────────────────────────────────────┐
│                         ROS2 JAZZY                           │
│                                                              │
│  /scan   → SLAM Toolbox → /map + TF map→odom               │
│  /tf     → TF tree: odom→base_footprint→base_link→base_scan │
│  /odom   → NAV2 (odometría)                                  │
│  /cmd_vel← NAV2 controller / teleop                         │
│                                                              │
│  NAV2 stack:                                                 │
│  ├── AMCL         → localización con partículas             │
│  ├── planner      → camino global (Dijkstra/A*)             │
│  ├── controller   → velocidades locales (DWB)               │
│  └── bt_navigator → coordina todo con Behavior Trees        │
└─────────────────────────────────────────────────────────────┘
```

### Por qué gpu_lidar necesita bridge
En Gazebo Classic (ROS Noetic) los sensores usaban plugins `gazebo_ros_*` que escribían directamente en ROS. En **Gazebo Harmonic los sensores son nativos** — publican en el bus interno de Gazebo. El `ros_gz_bridge` actúa de traductor: suscribe `/scan` en Gazebo y republica `sensor_msgs/LaserScan` en ROS2.

### Cadena TF completa
```
map ──(SLAM Toolbox)──► odom ──(DiffDrive bridge)──► base_footprint
                                                          │
                                               (robot_state_publisher)
                                                          │
                              ┌───────────────────────────┤
                              ▼                           ▼
                          base_scan                   base_link
                        (LiDAR frame)              wheel_*/camera_*
```

---

## Diagnóstico rápido

```bash
# ¿Llega el scan a ROS? (~1.7 Hz es normal con RTF 30%)
ros2 topic hz /scan

# ¿Está SLAM activo?
ros2 lifecycle get /slam_toolbox   # debe decir "active [4]"

# ¿Existe el frame map?
ros2 run tf2_ros tf2_echo map odom

# ¿Qué topics hay?
ros2 topic list

# ¿Qué publica Gazebo?
gz topic -l
```

---

## Prácticas del curso

| Carpeta | Descripción |
|---------|-------------|
| `diff_robot_urdf/` | CP1 — URDF robot diferencial, simulación básica |
| `image_folder_publisher/` | CP1 — Nodo publisher de imágenes desde carpeta |
| `sentiment_ros2/` | CP2 — Análisis de sentimiento con ROS2 |
| `tb3_description/` + `tb3_bringup/` | Simulación completa TurtleBot3 con SLAM y NAV2 |
