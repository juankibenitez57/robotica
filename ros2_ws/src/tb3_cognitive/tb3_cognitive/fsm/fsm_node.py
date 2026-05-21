"""
fsm/fsm_node.py — Nodo ROS2 con FSM cognitiva robusta (Fase 5)

Novedades Fase 5:
  - Detección visual YOLO: suscripción a /detected_objects durante SEARCHING
  - TARGET_FOUND: SEARCHING + target_found → APPROACHING (visual servoing)
  - _do_visual_approach: control proporcional cmd_vel para centrar y avanzar
  - _do_approach: espera primera detección y hace approach directo
  - _goal_result_cb: no transiciona desde APPROACHING si la cancelación
    fue disparada por detección (no por stop del usuario)

Topics:
  Sub:  /cognitive/command   (tb3_msgs/CognitiveCommand)
  Sub:  /detected_objects    (tb3_msgs/DetectedObject)    ← Fase 5
  Pub:  /cognitive/status    (std_msgs/String)
  Pub:  /cognitive/fsm_state (std_msgs/String)
  Act:  /navigate_to_pose    (nav2_msgs/action/NavigateToPose)
  Pub:  /cmd_vel             (geometry_msgs/Twist)

Parámetros:
  waypoints_file        — ruta al YAML con coordenadas
  nav_timeout           — segundos para esperar servidor NAV2 (default 5.0)
  max_abs_coordinate    — límite espacial preventivo en metros (default 20.0)
  confidence_threshold  — confianza mínima para ejecutar comando (default 0.35)
  max_nav_time          — watchdog: segundos máximos por goal antes de recovery (default 120.0)
  max_retries           — reintentos automáticos tras recovery (default 1)
"""

import math
import os
import threading
import time
import unicodedata
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from tb3_msgs.msg import CognitiveCommand, DetectedObject

from .states import RobotFSM, State
from .memory import CognitiveMemory


# Mapeo: target NLP normalizado → etiquetas YOLO COCO esperadas (normalizadas)
_YOLO_LABELS: dict[str, set[str]] = {
    'botella':  {'bottle'},
    'bottle':   {'bottle'},
    'persona':  {'person'},
    'person':   {'person'},
    'gente':    {'person'},
    'silla':    {'chair'},
    'chair':    {'chair'},
    'mesa':     {'dining_table'},
    'table':    {'dining_table'},
    'taza':     {'cup'},
    'cup':      {'cup'},
    'vaso':     {'cup'},
    'sofa':     {'couch'},
    'couch':    {'couch'},
    'comida':   {'banana', 'apple', 'sandwich', 'orange', 'pizza', 'hot_dog'},
    'perro':    {'dog'},
    'dog':      {'dog'},
    'gato':     {'cat'},
    'cat':      {'cat'},
    'laptop':   {'laptop'},
    'ordenador': {'laptop'},
}


# ── Rutas de búsqueda semántica ───────────────────────────────────────────────
# Mapea categorías de objeto → orden de habitaciones a visitar
_SEARCH_ROUTES: dict[str, list[str]] = {
    'default':  ['kitchen', 'living_room', 'dining_room', 'hallway', 'bedroom'],
    # Comida / bebida
    'botella':  ['kitchen', 'dining_room', 'living_room'],
    'bottle':   ['kitchen', 'dining_room', 'living_room'],
    'comida':   ['kitchen', 'dining_room'],
    'food':     ['kitchen', 'dining_room'],
    'vaso':     ['kitchen', 'dining_room'],
    'taza':     ['kitchen', 'dining_room'],
    # Personas
    'persona':  ['living_room', 'bedroom', 'kitchen', 'hallway'],
    'person':   ['living_room', 'bedroom', 'kitchen', 'hallway'],
    'gente':    ['living_room', 'bedroom', 'kitchen'],
    # Muebles
    'silla':    ['living_room', 'dining_room', 'bedroom'],
    'chair':    ['living_room', 'dining_room', 'bedroom'],
    'mesa':     ['dining_room', 'kitchen', 'living_room'],
    'table':    ['dining_room', 'kitchen', 'living_room'],
    'sofa':     ['living_room'],
}


class CognitiveFSMNode(Node):

    def __init__(self):
        super().__init__('cognitive_fsm_node')

        # ── Parámetros ────────────────────────────────────────────────────────
        waypoints_file = self.declare_parameter(
            'waypoints_file', '').get_parameter_value().string_value
        self._nav_timeout = self.declare_parameter(
            'nav_timeout', 5.0).get_parameter_value().double_value
        self._max_abs_coordinate = self.declare_parameter(
            'max_abs_coordinate', 20.0).get_parameter_value().double_value
        self._confidence_threshold = self.declare_parameter(
            'confidence_threshold', 0.35).get_parameter_value().double_value
        self._max_nav_time = self.declare_parameter(
            'max_nav_time', 120.0).get_parameter_value().double_value
        self._max_retries = self.declare_parameter(
            'max_retries', 1).get_parameter_value().integer_value

        # ── Waypoints + aliases ───────────────────────────────────────────────
        self._waypoints, self._aliases = self._load_waypoints(waypoints_file)
        self.get_logger().info(
            f'Waypoints cargados: {list(self._waypoints.keys())}')
        self.get_logger().info(
            f'Aliases cargados: {len(self._aliases)}  '
            f'Confidence threshold: {self._confidence_threshold:.2f}')

        # ── FSM + memoria ─────────────────────────────────────────────────────
        self._fsm = RobotFSM(on_transition=self._on_state_change)
        self._memory = CognitiveMemory()
        self._fsm_lock = threading.Lock()

        # ── Goal handle + evento de sincronización para patrol/search ─────────
        self._current_goal_handle = None
        self._goal_done_event: threading.Event | None = None
        self._goal_done_outcome: list = ['failed']

        # ── Detección visual (Fase 5) ─────────────────────────────────────────
        self._search_target_label: str | None = None   # activa filtro detección
        self._last_detection: DetectedObject | None = None

        # ── Watchdog de navegación ────────────────────────────────────────────
        self._nav_watchdog_timer = None

        # ── Retry tras recovery ───────────────────────────────────────────────
        self._retry_count   = 0
        self._last_nav_action: str | None = None
        self._last_nav_target: str | None = None

        # ── Publishers ────────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/cognitive/status', 10)
        self._fsm_state_pub = self.create_publisher(String, '/cognitive/fsm_state', 10)

        # ── NAV2 action client ────────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Costmap clear services ────────────────────────────────────────────
        self._global_clear = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self._local_clear = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        # ── cmd_vel ───────────────────────────────────────────────────────────
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Suscripciones ─────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            CognitiveCommand, '/cognitive/command',
            self._command_callback, 10)
        self._detection_sub = self.create_subscription(
            DetectedObject, '/detected_objects',
            self._detection_callback, 10)

        self._publish_fsm_state(State.IDLE)
        self.get_logger().info(
            'CognitiveFSMNode activo — Fase 5\n'
            '  Sub: /cognitive/command | /detected_objects\n'
            '  Pub: /cognitive/status | /cognitive/fsm_state\n'
            '  Act: /navigate_to_pose\n'
            f'  Waypoints: {len(self._waypoints)}  '
            f'Threshold: {self._confidence_threshold:.2f}')

    # ── Carga de waypoints ────────────────────────────────────────────────────

    def _load_waypoints(self, filepath: str) -> tuple[dict, dict]:
        if not filepath or not os.path.exists(filepath):
            self.get_logger().warn(f'waypoints_file no existe: {filepath!r}')
            return {}, {}
        with open(filepath) as f:
            data = yaml.safe_load(f) or {}

        meta = data.get('metadata', {})
        if 'max_abs_coordinate' in meta:
            self._max_abs_coordinate = float(meta['max_abs_coordinate'])

        raw_waypoints = data.get('waypoints', {})
        raw_aliases   = data.get('aliases', {})

        aliases: dict[str, str] = {
            self._norm(alias): self._norm(target)
            for alias, target in raw_aliases.items()
        }
        valid: dict[str, dict] = {}
        for name, wp in raw_waypoints.items():
            nname = self._norm(name)
            ok, reason = self._validate_wp(nname, wp)
            if not ok:
                self.get_logger().error(
                    f'Waypoint descartado: {name!r} — {reason}')
                continue
            valid[nname] = wp
            aliases.setdefault(nname, nname)

        return valid, aliases

    # ── Callback principal ────────────────────────────────────────────────────

    def _command_callback(self, msg: CognitiveCommand) -> None:
        self._memory.record_command(msg.action, msg.target, msg.confidence)

        self.get_logger().info(
            f'[CMD] action={msg.action!r}  target={msg.target!r}  '
            f'conf={msg.confidence:.2f}')

        with self._fsm_lock:
            # ── Stop siempre se procesa ───────────────────────────────────────
            if msg.action == 'stop':
                self._handle_stop()
                return

            # ── Reset manual ──────────────────────────────────────────────────
            if msg.action == 'reset':
                ok = self._fsm.trigger('reset')
                self._publish_status(
                    'FSM → IDLE.' if ok
                    else f'Reset no válido desde {self._fsm.state.value}.')
                return

            # ── Confidence gating → UNKNOWN ───────────────────────────────────
            if (msg.confidence < self._confidence_threshold
                    and msg.action not in ('stop', 'reset', 'report')):
                self.get_logger().warn(
                    f'[UNKNOWN] conf={msg.confidence:.2f} < '
                    f'{self._confidence_threshold:.2f}  '
                    f'action={msg.action!r} — comando ambiguo.')
                self._fsm.trigger('unknown')   # IDLE → UNKNOWN
                self._publish_status(
                    f'Comando ambiguo (conf={msg.confidence:.2f}): '
                    f'reformula la instrucción.')
                self._fsm.trigger('reset')     # UNKNOWN → IDLE (transiente)
                return

            # ── Auto-recovery en ERROR ────────────────────────────────────────
            if self._fsm.state == State.ERROR:
                self._publish_status('En ERROR: ejecutando recovery automático...')
                threading.Thread(
                    target=self._recover_then_dispatch,
                    args=(msg.action, msg.target),
                    daemon=True,
                ).start()
                return

            # ── Auto-reset desde STOPPED ──────────────────────────────────────
            if self._fsm.state == State.STOPPED:
                self._fsm.trigger('reset')

            # ── Rechaza si está ocupado ───────────────────────────────────────
            if self._fsm.is_busy():
                self.get_logger().warn(
                    f'[BUSY] {self._fsm.state.value}: '
                    f'comando {msg.action!r} ignorado. Di "para" primero.')
                self._publish_status(
                    f'Ocupado ({self._fsm.state.value}). Di "para" para detener.')
                return

            # ── Transición FSM ────────────────────────────────────────────────
            if not self._fsm.trigger(msg.action):
                self.get_logger().warn(
                    f'Transición inválida: {self._fsm.state.value} + {msg.action!r}')
                return

        threading.Thread(
            target=self._dispatch,
            args=(msg.action, msg.target),
            daemon=True,
        ).start()

    # ── Dispatch ──────────────────────────────────────────────────────────────

    def _dispatch(self, action: str, target: str) -> None:
        dispatch_map = {
            'navigate': lambda: self._do_navigate(target),
            'explore':  self._do_explore,
            'search':   lambda: self._do_search(target),
            'approach': lambda: self._do_approach(target),
            'report':   self._do_report,
        }
        fn = dispatch_map.get(action)
        if fn:
            fn()

    # ── Acciones ──────────────────────────────────────────────────────────────

    def _do_navigate(self, target: str) -> None:
        resolved = self._find_waypoint(target)
        if resolved is None:
            msg = (f'Waypoint desconocido: {target!r}. '
                   f'Disponibles: {list(self._waypoints.keys())}')
            self.get_logger().warn(msg)
            self._publish_status(msg)
            self._memory.record_failure(f'waypoint desconocido: {target!r}')
            with self._fsm_lock:
                self._fsm.trigger('goal_failed')
            self._start_recovery()
            return

        wp_name, wp = resolved
        x, y   = float(wp['x']), float(wp['y'])
        yaw    = float(wp.get('yaw', 0.0))

        self.get_logger().info(
            f'[NAV] Intent={target!r}  Grounding={wp_name!r}  '
            f'Coords=({x:.2f}, {y:.2f})  yaw={yaw:.2f}')
        self._publish_status(
            f'Navegando: {target!r} → {wp_name} ({x:.2f}, {y:.2f})')
        self._memory.record_navigation(wp_name, x, y)

        # Guarda contexto para posible retry
        self._last_nav_action = 'navigate'
        self._last_nav_target = target

        self._start_nav_watchdog()
        outcome = self._send_nav_goal_blocking(x, y, yaw=yaw, label=wp_name)
        self._stop_nav_watchdog()

        if outcome == 'succeeded':
            self._retry_count = 0
            self._memory.record_success(wp_name)

    def _do_explore(self) -> None:
        exclude = {'origin', 'origen'}
        patrol = [n for n in self._waypoints if n not in exclude]
        if not patrol:
            self._publish_status('No hay waypoints para explorar.')
            with self._fsm_lock:
                self._fsm.trigger('goal_succeeded')
            return

        self.get_logger().info(f'[EXPLORE] Patrulla: {patrol}')
        self._publish_status(f'Exploración secuencial: {len(patrol)} waypoints')

        for wp_name in patrol:
            with self._fsm_lock:
                if self._fsm.state not in (State.EXPLORING,):
                    return  # stop recibido

            wp  = self._waypoints[wp_name]
            x   = float(wp['x'])
            y   = float(wp['y'])
            yaw = float(wp.get('yaw', 0.0))

            self.get_logger().info(
                f'[EXPLORE] → {wp_name} ({x:.2f}, {y:.2f})')
            self._publish_status(f'Explorando → {wp_name} ({x:.2f}, {y:.2f})')

            outcome = self._send_nav_goal_blocking(x, y, yaw=yaw, label=wp_name)

            if outcome == 'cancelled':
                return
            if outcome == 'failed':
                self.get_logger().warn(
                    f'[EXPLORE] {wp_name} inaccesible — saltando al siguiente.')
                with self._fsm_lock:
                    # Reset simple ERROR→IDLE→EXPLORING para continuar patrulla.
                    # NO llamar a _start_recovery() aquí: evita race condition
                    # con el patrol loop que sigue corriendo en este mismo thread.
                    if self._fsm.state == State.ERROR:
                        self._fsm.trigger('reset')    # ERROR → IDLE
                        self._fsm.trigger('explore')  # IDLE  → EXPLORING
                continue  # salta al siguiente waypoint

        with self._fsm_lock:
            if self._fsm.state == State.EXPLORING:
                self._publish_status('Exploración completada.')
                self._fsm.trigger('goal_succeeded')

    def _do_search(self, target: str) -> None:
        t_norm = self._norm(target)
        route_keys = list(_SEARCH_ROUTES.keys())
        route = next(
            (_SEARCH_ROUTES[k] for k in route_keys if k in t_norm),
            _SEARCH_ROUTES['default']
        )
        route = [r for r in route if r in self._waypoints]
        if not route:
            route = [n for n in self._waypoints if n not in ('origin', 'origen')]

        self.get_logger().info(
            f'[SEARCH] Buscando: {target!r}  Ruta: {route}')
        self._publish_status(
            f'Buscando {target!r}: ruta {" → ".join(route)}')

        # Activa filtro de detección YOLO
        self._search_target_label = t_norm
        self._last_detection = None

        for wp_name in route:
            with self._fsm_lock:
                cur = self._fsm.state
                if cur == State.APPROACHING:
                    break  # detección antes de navegar
                if cur not in (State.SEARCHING,):
                    self._search_target_label = None
                    return

            wp  = self._waypoints[wp_name]
            x   = float(wp['x'])
            y   = float(wp['y'])
            yaw = float(wp.get('yaw', 0.0))

            self.get_logger().info(
                f'[SEARCH] Inspeccionando {wp_name} ({x:.2f}, {y:.2f})')
            self._publish_status(f'Buscando {target!r} en {wp_name}...')

            outcome = self._send_nav_goal_blocking(x, y, yaw=yaw, label=wp_name)

            with self._fsm_lock:
                cur = self._fsm.state
                if cur == State.APPROACHING:
                    break  # detección durante navegación

            if outcome == 'cancelled':
                with self._fsm_lock:
                    if self._fsm.state == State.APPROACHING:
                        break  # cancel disparado por detección
                self._search_target_label = None
                return

            if outcome == 'failed':
                self.get_logger().warn(
                    f'[SEARCH] {wp_name} inaccesible — saltando.')
                with self._fsm_lock:
                    if self._fsm.state == State.ERROR:
                        self._fsm.trigger('reset')
                        self._fsm.trigger('search')
                continue

            # Pausa en el waypoint para que YOLO escanee la habitación (2 s)
            self._publish_status(f'Escaneando {wp_name}...')
            for _ in range(20):
                with self._fsm_lock:
                    if self._fsm.state == State.APPROACHING:
                        break
                time.sleep(0.1)

            with self._fsm_lock:
                if self._fsm.state == State.APPROACHING:
                    break

        # ── Resultado del bucle ────────────────────────────────────────────────
        with self._fsm_lock:
            cur = self._fsm.state

        if cur == State.APPROACHING:
            # Target encontrado — continuar con acercamiento visual en este thread
            lbl = self._search_target_label or target
            self.get_logger().info(
                f'[SEARCH] {lbl!r} detectado! Iniciando acercamiento visual.')
            self._publish_status(
                f'{lbl!r} detectado. Acercándome...')
            self._do_visual_approach()
        else:
            self._search_target_label = None
            with self._fsm_lock:
                if self._fsm.state == State.SEARCHING:
                    self._publish_status(
                        f'Búsqueda de {target!r} completada — no detectado visualmente.')
                    self._fsm.trigger('goal_succeeded')

    def _do_approach(self, target: str) -> None:
        self._search_target_label = self._norm(target)
        self._last_detection = None
        self.get_logger().info(
            f'[APPROACH] Esperando detección de {target!r}...')
        self._publish_status(
            f'Esperando {target!r} en cámara (10 s)...')

        # Espera hasta 10 s a que YOLO detecte el objetivo
        for _ in range(100):
            if self._last_detection is not None:
                break
            with self._fsm_lock:
                if self._fsm.state != State.APPROACHING:
                    return
            time.sleep(0.1)

        if self._last_detection is None:
            self._publish_status(
                f'{target!r} no visible. Apunta la cámara al objeto.')
            self._search_target_label = None
            with self._fsm_lock:
                self._fsm.trigger('goal_succeeded')
            return

        self._do_visual_approach()

    def _do_report(self) -> None:
        nav_ok  = self._nav_client.server_is_ready()
        summary = self._memory.summary()
        msg = (
            f'FSM={self._fsm.state.value}  '
            f'NAV2={"OK" if nav_ok else "NO"}  '
            f'waypoints={len(self._waypoints)}\n{summary}')
        self.get_logger().info(f'[REPORT] {msg}')
        self._publish_status(msg)
        with self._fsm_lock:
            self._fsm.trigger('done')

    # ── Detección visual YOLO (Fase 5) ────────────────────────────────────────

    def _detection_callback(self, msg: DetectedObject) -> None:
        if not self._search_target_label:
            return
        with self._fsm_lock:
            cur = self._fsm.state
            if cur == State.APPROACHING:
                # Approach en curso — actualiza detección para el servoing
                if self._detection_matches_target(msg.label, self._search_target_label):
                    self._last_detection = msg
                return
            if cur != State.SEARCHING:
                return
            if not self._detection_matches_target(msg.label, self._search_target_label):
                return
            # Primera detección durante búsqueda → transiciona
            self._last_detection = msg
            if not self._fsm.trigger('target_found'):  # SEARCHING → APPROACHING
                return

        self.get_logger().info(
            f'[VISION] {msg.label!r} detectado!  '
            f'conf={msg.confidence:.2f}  cx={msg.center_x_norm:.2f}  '
            f'area={msg.area_norm:.3f}')
        self._publish_status(
            f'{msg.label!r} detectado (conf={msg.confidence:.2f})!')

        # Cancela navegación activa para que el search loop despierte
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
        else:
            self._signal_goal_done('cancelled')

    def _detection_matches_target(self, yolo_label: str, target: str) -> bool:
        t = self._norm(target)
        normed_label = self._norm(yolo_label)
        labels = _YOLO_LABELS.get(t)
        if labels:
            return normed_label in labels
        return t in normed_label or normed_label in t

    def _do_visual_approach(self) -> None:
        """
        Control proporcional para centrar el objeto en cámara y avanzar.
        Corre en el thread del search/approach — usa time.sleep() como rate.
        """
        KP_ANG   = 1.2
        KP_LIN   = 0.5
        MAX_LIN  = 0.15   # m/s
        AREA_TGT = 0.15   # detener cuando el objeto ocupa ≥15% del frame
        TIMEOUT  = 30.0

        label = self._search_target_label or 'objeto'
        self.get_logger().info(
            f'[APPROACH] Servoing visual hacia {label!r} '
            f'(área objetivo {AREA_TGT*100:.0f}%)')
        t_start = time.time()

        while rclpy.ok():
            if time.time() - t_start > TIMEOUT:
                self.get_logger().warn(
                    f'[APPROACH] Timeout {TIMEOUT:.0f}s — '
                    f'{label!r} perdido o inalcanzable.')
                break

            with self._fsm_lock:
                if self._fsm.state != State.APPROACHING:
                    return  # stop externo

            det    = self._last_detection
            twist  = Twist()

            if det is None:
                # Sin detección — girar despacio para buscar
                twist.angular.z = 0.25
            else:
                error_x = det.center_x_norm - 0.5   # >0 → objeto a la derecha
                area    = det.area_norm

                twist.angular.z = -KP_ANG * error_x  # CCW para centrar
                if area < AREA_TGT:
                    advance = KP_LIN * (AREA_TGT - area)
                    twist.linear.x = min(advance, MAX_LIN)

                if area >= AREA_TGT and abs(error_x) < 0.10:
                    self.get_logger().info(
                        f'[APPROACH] {label!r} alcanzado  '
                        f'area={area:.3f}  err_x={error_x:.3f}')
                    break

            self._cmd_vel_pub.publish(twist)
            time.sleep(0.10)  # 10 Hz

        self._zero_velocity()
        self._search_target_label = None
        self._last_detection      = None

        msg_txt = (f'¡{label!r} encontrado y alcanzado! '
                   'Robot parado frente al objetivo.')
        self.get_logger().info(f'[APPROACH] {msg_txt}')
        self._publish_status(msg_txt)
        self._memory.record_success(label)

        with self._fsm_lock:
            self._fsm.trigger('goal_succeeded')  # APPROACHING → IDLE

    # ── Stop / cancela goal activo ────────────────────────────────────────────

    def _handle_stop(self) -> None:
        if not self._fsm.trigger('stop'):
            # Ya en STOPPED/IDLE — solo publica Twist=0
            self._zero_velocity()
            self._publish_status('Robot ya detenido.')
            return

        # STOPPING: envía cancel al goal activo
        if self._current_goal_handle is not None:
            self.get_logger().info('[STOP] cancel_goal_async enviado.')
            self._current_goal_handle.cancel_goal_async()
        else:
            # No hay goal activo — transición directa a STOPPED
            self._fsm.trigger('goal_cancelled')

        self._zero_velocity()
        self._publish_status('Deteniendo... (cancelando goal NAV2)')

    def _zero_velocity(self) -> None:
        twist = Twist()
        for _ in range(5):
            self._cmd_vel_pub.publish(twist)

    # ── Watchdog de navegación ────────────────────────────────────────────────

    def _start_nav_watchdog(self) -> None:
        self._stop_nav_watchdog()
        self._nav_watchdog_timer = self.create_timer(
            self._max_nav_time, self._nav_watchdog_cb)

    def _stop_nav_watchdog(self) -> None:
        if self._nav_watchdog_timer is not None:
            self._nav_watchdog_timer.cancel()
            self._nav_watchdog_timer = None

    def _nav_watchdog_cb(self) -> None:
        self._stop_nav_watchdog()
        with self._fsm_lock:
            if self._fsm.state not in (
                    State.NAVIGATING, State.EXPLORING, State.SEARCHING):
                return
        self.get_logger().error(
            f'[WATCHDOG] Timeout {self._max_nav_time:.0f}s — '
            'robot posiblemente atascado. Iniciando recovery.')
        self._publish_status(
            f'Watchdog: navegación superó {self._max_nav_time:.0f}s. Recovery.')
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
        with self._fsm_lock:
            self._fsm.trigger('goal_failed')
        self._start_recovery()
        self._signal_goal_done('failed')

    # ── Recovery ──────────────────────────────────────────────────────────────

    def _start_recovery(self) -> None:
        threading.Thread(target=self._do_recovery_with_retry, daemon=True).start()

    def _do_recovery_with_retry(self) -> None:
        ok = self._do_recovery()
        if not ok:
            return
        # Retry automático si hay un goal previo y quedan intentos
        if (self._last_nav_action is not None
                and self._retry_count < self._max_retries):
            self._retry_count += 1
            action = self._last_nav_action
            target = self._last_nav_target or ''
            self.get_logger().info(
                f'[RETRY] Intento {self._retry_count}/{self._max_retries}: '
                f'{action!r} → {target!r}')
            self._publish_status(
                f'Recovery OK. Reintentando ({self._retry_count}/{self._max_retries}): '
                f'{action} → {target!r}')
            with self._fsm_lock:
                if not self._fsm.trigger(action):
                    return
            self._dispatch(action, target)
        else:
            if self._retry_count >= self._max_retries:
                self.get_logger().warn(
                    f'[RETRY] Máximo de reintentos alcanzado '
                    f'({self._max_retries}). FSM en IDLE.')
                self._publish_status(
                    f'Reintentos agotados ({self._max_retries}). '
                    'Esperando nuevo comando.')
            self._retry_count = 0
            self._last_nav_action = None
            self._last_nav_target = None

    def _recover_then_dispatch(self, action: str, target: str) -> None:
        if not self._do_recovery():
            return
        with self._fsm_lock:
            if not self._fsm.trigger(action):
                self.get_logger().warn(
                    f'Transición inválida tras recovery: '
                    f'{self._fsm.state.value} + {action!r}')
                return
        self._dispatch(action, target)

    def _do_recovery(self) -> bool:
        with self._fsm_lock:
            if self._fsm.state == State.RECOVERY:
                return False
            if self._fsm.state != State.ERROR:
                return True
            if not self._fsm.trigger('recovery_start'):
                return False

        self._memory.record_recovery()
        self._publish_status('Recovery: limpiando costmaps NAV2...')
        self.get_logger().warn('[RECOVERY] clear_costmaps iniciado.')

        ok = True
        for name, client in (
            ('global_costmap', self._global_clear),
            ('local_costmap',  self._local_clear),
        ):
            if not client.wait_for_service(timeout_sec=1.5):
                self.get_logger().warn(f'[RECOVERY] {name} no disponible.')
                ok = False
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            event  = threading.Event()
            future.add_done_callback(lambda _: event.set())
            if not event.wait(timeout=3.0) or future.exception():
                self.get_logger().warn(f'[RECOVERY] fallo limpiando {name}.')
                ok = False
            else:
                self.get_logger().info(f'[RECOVERY] {name} limpio ✓')

        with self._fsm_lock:
            self._fsm.trigger('recovery_done' if ok else 'recovery_failed')

        status = 'Recovery completado → IDLE.' if ok \
            else 'Recovery incompleto. Revisa NAV2.'
        self._publish_status(status)
        return ok

    # ── NAV2 goal — versión bloqueante para patrol/search ────────────────────

    def _send_nav_goal_blocking(
        self, x: float, y: float, yaw: float = 0.0, label: str = ''
    ) -> str:
        """
        Envía un NavigateToPose goal y bloquea el hilo llamante hasta
        recibir el resultado. Devuelve 'succeeded' | 'failed' | 'cancelled'.
        Compatible con explore/search que ejecutan en un thread propio.
        """
        event = threading.Event()
        self._goal_done_event   = event
        self._goal_done_outcome = ['failed']

        self._send_nav_goal(x, y, yaw=yaw, label=label)
        event.wait()
        return self._goal_done_outcome[0]

    def _send_nav_goal(
        self, x: float, y: float, yaw: float = 0.0, label: str = ''
    ) -> None:
        if not self._nav_client.wait_for_server(
                timeout_sec=self._nav_timeout):
            msg = 'NAV2 no responde. Lanza navigation.launch.py primero.'
            self.get_logger().error(msg)
            self._publish_status(msg)
            with self._fsm_lock:
                self._fsm.trigger('goal_failed')
            self._start_recovery()
            self._signal_goal_done('failed')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id    = 'map'
        goal.pose.header.stamp       = self.get_clock().now().to_msg()
        goal.pose.pose.position.x    = x
        goal.pose.pose.position.y    = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'[NAV2] goal → {label} ({x:.2f}, {y:.2f})  Planner: active')
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('[NAV2] goal RECHAZADO.')
            self._publish_status('Goal rechazado por NAV2.')
            with self._fsm_lock:
                self._fsm.trigger('goal_failed')
            self._start_recovery()
            self._signal_goal_done('failed')
            return

        self._current_goal_handle = handle
        self.get_logger().info('[NAV2] Goal aceptado — esperando resultado...')
        handle.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future) -> None:
        self._current_goal_handle = None
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('[NAV2] Goal succeeded ✓')
            self._publish_status('Llegué al destino.')
            with self._fsm_lock:
                # No tocar si ya estamos en APPROACHING (detección disparó transición)
                if self._fsm.state != State.APPROACHING:
                    self._fsm.trigger('goal_succeeded')
            self._signal_goal_done('succeeded')

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('[NAV2] Goal cancelado.')
            with self._fsm_lock:
                cur = self._fsm.state
                if cur == State.APPROACHING:
                    # Cancel disparado por detección YOLO — APPROACHING es correcto
                    pass
                else:
                    self._fsm.trigger('goal_cancelled')
                    if self._fsm.state == State.STOPPED:
                        self._fsm.trigger('reset')
            self._zero_velocity()
            if cur != State.APPROACHING:
                self._publish_status('Robot detenido. FSM en IDLE.')
            self._signal_goal_done('cancelled')

        else:
            self.get_logger().warn(f'[NAV2] Goal FALLIDO (status={status}).')
            self._publish_status(f'No llegué al destino (status={status}).')
            self._memory.record_failure(f'NAV2 status={status}')
            with self._fsm_lock:
                cur = self._fsm.state
                prev = cur
                if cur != State.APPROACHING:
                    self._fsm.trigger('goal_failed')
            # Recovery automático solo para NAVIGATING — explore/search
            # manejan sus propios fallos de waypoint inline para evitar
            # race conditions entre el recovery thread y el patrol loop.
            if prev == State.NAVIGATING:
                self._start_recovery()
            self._signal_goal_done('failed')

    def _signal_goal_done(self, outcome: str) -> None:
        if self._goal_done_event is not None:
            self._goal_done_outcome[0] = outcome
            self._goal_done_event.set()
            self._goal_done_event = None

    # ── FSM callback ─────────────────────────────────────────────────────────

    def _on_state_change(self, old: State, event: str, new: State) -> None:
        self.get_logger().info(
            f'[FSM] {old.value} ──{event}──→ {new.value}')
        self._publish_fsm_state(new)

    # ── Utilidades ────────────────────────────────────────────────────────────

    def _find_waypoint(self, target: str) -> tuple[str, dict] | None:
        t = self._norm(target)
        if t in self._aliases:
            wp_name = self._aliases[t]
            wp = self._waypoints.get(wp_name)
            if wp is not None:
                return wp_name, wp
        for alias, wp_name in self._aliases.items():
            if alias and alias in t:
                wp = self._waypoints.get(wp_name)
                if wp is not None:
                    return wp_name, wp
        return None

    def _norm(self, value: str) -> str:
        text = str(value or '').strip().lower().replace('_', ' ')
        text = ''.join(
            c for c in unicodedata.normalize('NFD', text)
            if unicodedata.category(c) != 'Mn'
        )
        return ' '.join(text.split()).replace(' ', '_')

    def _validate_wp(self, name: str, wp: dict) -> tuple[bool, str]:
        if not isinstance(wp, dict):
            return False, 'no es dict'
        for field in ('x', 'y'):
            if field not in wp:
                return False, f'falta {field!r}'
            try:
                v = float(wp[field])
            except (TypeError, ValueError):
                return False, f'{field} no numérico'
            if not math.isfinite(v):
                return False, f'{field} no finito'
            if abs(v) > self._max_abs_coordinate:
                return False, (
                    f'{field}={v:.2f} excede ±{self._max_abs_coordinate:.1f}m')
        if not name:
            return False, 'nombre vacío'
        return True, 'ok'

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _publish_fsm_state(self, state: State) -> None:
        msg = String()
        msg.data = state.value
        self._fsm_state_pub.publish(msg)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CognitiveFSMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
