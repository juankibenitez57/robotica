"""
fsm/fsm_node.py — Nodo ROS2 con FSM cognitiva (Fase 3)

Mejora sobre Fase 2 (agent_node):
  - Estado explícito: el robot sabe dónde está (IDLE/NAVIGATING/ERROR/...)
  - Goal tracking: callbacks encadenados para recibir resultado de NAV2
  - Comandos ignorados si el robot ya está ocupado (evita goals superpuestos)
  - /cognitive/fsm_state publica el estado actual en tiempo real

Topics:
  Sub:  /cognitive/command   (tb3_msgs/CognitiveCommand)
  Pub:  /cognitive/status    (std_msgs/String)   — resultado legible
  Pub:  /cognitive/fsm_state (std_msgs/String)   — estado FSM actual
  Act:  /navigate_to_pose    (nav2_msgs/action/NavigateToPose)
  Pub:  /cmd_vel             (geometry_msgs/Twist)

Parámetros:
  waypoints_file  — ruta al YAML con coordenadas
  nav_timeout     — segundos para esperar servidor NAV2 (default 5.0)
"""

import os
import yaml
import math
import random
import threading
import unicodedata

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from tb3_msgs.msg import CognitiveCommand

from .states import RobotFSM, State


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

        # ── Waypoints ─────────────────────────────────────────────────────────
        self._waypoints, self._aliases = self._load_waypoints(waypoints_file)
        self.get_logger().info(
            f'Waypoints cargados: {list(self._waypoints.keys())}')
        self.get_logger().info(
            f'Aliases semánticos cargados: {len(self._aliases)}')

        # ── FSM ───────────────────────────────────────────────────────────────
        self._fsm = RobotFSM(on_transition=self._on_state_change)
        self._current_goal_handle = None
        self._fsm_lock = threading.Lock()

        # ── Publishers ────────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(
            String, '/cognitive/status', 10)
        self._fsm_state_pub = self.create_publisher(
            String, '/cognitive/fsm_state', 10)

        # ── NAV2 action client ────────────────────────────────────────────────
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self._global_clear_client = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self._local_clear_client = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        # ── cmd_vel para stop ─────────────────────────────────────────────────
        self._cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        # ── Suscripción ───────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            CognitiveCommand, '/cognitive/command',
            self._command_callback, 10)

        self._publish_fsm_state(State.IDLE)
        self.get_logger().info(
            'CognitiveFSMNode activo — Fase 3\n'
            '  Sub: /cognitive/command\n'
            '  Pub: /cognitive/status\n'
            '  Pub: /cognitive/fsm_state\n'
            '  Act: /navigate_to_pose')

    # ── Carga de waypoints ────────────────────────────────────────────────────

    def _load_waypoints(self, filepath: str) -> tuple[dict, dict]:
        if not filepath or not os.path.exists(filepath):
            self.get_logger().warn(
                f'waypoints_file no existe: {filepath!r}')
            return {}, {}
        with open(filepath) as f:
            data = yaml.safe_load(f) or {}

        metadata = data.get('metadata', {})
        if 'max_abs_coordinate' in metadata:
            self._max_abs_coordinate = float(metadata['max_abs_coordinate'])

        waypoints = data.get('waypoints', {})
        aliases = {
            self._normalize_location(alias): self._normalize_location(target)
            for alias, target in data.get('aliases', {}).items()
        }
        for name in waypoints:
            normalized = self._normalize_location(name)
            aliases.setdefault(normalized, normalized)

        valid_waypoints = {}
        for name, waypoint in waypoints.items():
            normalized_name = self._normalize_location(name)
            ok, reason = self._validate_waypoint(normalized_name, waypoint)
            if not ok:
                self.get_logger().error(
                    f'Waypoint inválido descartado: {name!r} — {reason}')
                continue
            valid_waypoints[normalized_name] = waypoint

        return valid_waypoints, aliases

    # ── Callback de comando ───────────────────────────────────────────────────

    def _command_callback(self, msg: CognitiveCommand) -> None:
        self.get_logger().info(
            f'Comando → action={msg.action!r}  '
            f'target={msg.target!r}  conf={msg.confidence:.2f}')

        with self._fsm_lock:
            # Stop cancela cualquier estado activo
            if msg.action == 'stop':
                self._handle_stop()
                return

            if msg.action == 'reset':
                if self._fsm.trigger('reset'):
                    self._publish_status('FSM reseteada a IDLE.')
                else:
                    self._publish_status(
                        f'Reset ignorado desde {self._fsm.state.value}.')
                return

            if self._fsm.state == State.ERROR:
                self._publish_status(
                    'FSM en ERROR: ejecutando recovery antes del nuevo comando.')
                threading.Thread(
                    target=self._recover_then_dispatch,
                    args=(msg.action, msg.target),
                    daemon=True,
                ).start()
                return

            if self._fsm.state == State.STOPPED:
                if not self._fsm.trigger('reset'):
                    self._publish_status('No pude salir de STOPPED.')
                    return

            # Si está ocupado, rechaza el nuevo comando
            if self._fsm.is_busy():
                self.get_logger().warn(
                    f'Robot ocupado en estado {self._fsm.state.value} — '
                    f'comando {msg.action!r} ignorado. Di "para" primero.')
                self._publish_status(
                    f'Ocupado ({self._fsm.state.value}): comando ignorado.')
                return

            # Intenta la transición
            if not self._fsm.trigger(msg.action):
                self.get_logger().warn(
                    f'Transición no válida: {self._fsm.state.value} + {msg.action!r}')
                return

        # Ejecuta en thread para no bloquear el executor de ROS2
        threading.Thread(
            target=self._dispatch,
            args=(msg.action, msg.target),
            daemon=True,
        ).start()

    # ── Dispatch por acción ───────────────────────────────────────────────────

    def _dispatch(self, action: str, target: str) -> None:
        if action == 'navigate':
            self._do_navigate(target)
        elif action == 'explore':
            self._do_explore()
        elif action == 'search':
            self._do_search(target)
        elif action == 'approach':
            self._do_approach(target)
        elif action == 'report':
            self._do_report()

    # ── Implementaciones ─────────────────────────────────────────────────────

    def _do_navigate(self, target: str) -> None:
        resolved = self._find_waypoint(target)
        if resolved is None:
            available = list(self._waypoints.keys())
            msg = (f'Waypoint desconocido: {target!r}. '
                   f'Disponibles: {available}')
            self.get_logger().warn(msg)
            self._publish_status(msg)
            with self._fsm_lock:
                self._fsm.trigger('goal_failed')
            self._start_recovery()
            return

        waypoint_name, waypoint = resolved
        x, y = float(waypoint['x']), float(waypoint['y'])
        yaw = float(waypoint.get('yaw', 0.0))
        ok, reason = self._validate_waypoint(waypoint_name, waypoint)
        if not ok:
            msg = f'Waypoint rechazado preventivamente: {waypoint_name} — {reason}'
            self.get_logger().error(msg)
            self._publish_status(msg)
            with self._fsm_lock:
                self._fsm.trigger('goal_failed')
            self._start_recovery()
            return

        self.get_logger().info(
            f'Grounding: target={target!r} → waypoint={waypoint_name!r}')
        self.get_logger().info(
            f'Coordinates: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')
        self._publish_status(
            f'Navegando a {target!r} → {waypoint_name} ({x:.2f}, {y:.2f})')
        self._send_nav_goal(x, y, yaw=yaw, label=waypoint_name)

    def _do_explore(self) -> None:
        points = [
            (1.5,  1.5), (-1.5,  1.5),
            (1.5, -1.5), (-1.5, -1.5),
            (0.0,  2.0), ( 2.0,  0.0),
        ]
        x, y = random.choice(points)
        self._publish_status(f'Explorando hacia ({x:.1f}, {y:.1f})')
        self._send_nav_goal(x, y, label=f'exploración ({x:.1f},{y:.1f})')

    def _do_search(self, target: str) -> None:
        self._publish_status(
            f'Buscando {target!r}: navegando al centro del mapa')
        self._send_nav_goal(0.0, 0.0, label=f'búsqueda de {target!r}')

    def _do_approach(self, target: str) -> None:
        msg = f'Approach hacia {target!r}: disponible en Fase 4 (YOLO).'
        self.get_logger().info(msg)
        self._publish_status(msg)
        with self._fsm_lock:
            self._fsm.trigger('goal_succeeded')

    def _do_report(self) -> None:
        nav_ok = self._nav_client.server_is_ready()
        wps = len(self._waypoints)
        state = self._fsm.state.value
        msg = (f'Estado FSM={state}  NAV2={"OK" if nav_ok else "NO"}  '
               f'waypoints={wps}  nodo=activo')
        self.get_logger().info(msg)
        self._publish_status(msg)
        with self._fsm_lock:
            self._fsm.trigger('done')

    def _handle_stop(self) -> None:
        # Cancela goal activo si existe
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

        # Publica Twist cero varias veces
        twist = Twist()
        for _ in range(5):
            self._cmd_vel_pub.publish(twist)

        self._fsm.trigger('stop')
        if self._fsm.state == State.STOPPED:
            self._fsm.trigger('reset')
        self._publish_status('Robot detenido. FSM lista en IDLE.')
        self.get_logger().info('Robot detenido (FSM → IDLE)')

    def _recover_then_dispatch(self, action: str, target: str) -> None:
        if not self._do_recovery():
            return
        with self._fsm_lock:
            if not self._fsm.trigger(action):
                self.get_logger().warn(
                    f'Transición no válida tras recovery: '
                    f'{self._fsm.state.value} + {action!r}')
                return
        self._dispatch(action, target)

    def _start_recovery(self) -> None:
        threading.Thread(target=self._do_recovery, daemon=True).start()

    def _do_recovery(self) -> bool:
        with self._fsm_lock:
            if self._fsm.state == State.RECOVERY:
                return False
            if self._fsm.state != State.ERROR:
                return True
            if not self._fsm.trigger('recovery_start'):
                return False

        self._publish_status('Recovery: limpiando costmaps NAV2.')
        self.get_logger().warn('Recovery: clear_costmaps → reset_navigation')

        ok = True
        for name, client in (
            ('global_costmap', self._global_clear_client),
            ('local_costmap', self._local_clear_client),
        ):
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(
                    f'Recovery: servicio {name} no disponible.')
                ok = False
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            event = threading.Event()
            future.add_done_callback(lambda _: event.set())
            if not event.wait(timeout=2.0):
                self.get_logger().warn(f'Recovery: timeout limpiando {name}.')
                ok = False
            elif future.exception() is not None:
                self.get_logger().warn(
                    f'Recovery: fallo limpiando {name}: {future.exception()}')
                ok = False
            else:
                self.get_logger().info(f'Recovery: {name} limpio.')

        with self._fsm_lock:
            self._fsm.trigger('recovery_done' if ok else 'recovery_failed')

        if ok:
            self._publish_status('Recovery completado. FSM en IDLE.')
        else:
            self._publish_status(
                'Recovery incompleto. Revisa servicios de costmap NAV2.')
        return ok

    # ── NAV2 goal tracking ────────────────────────────────────────────────────

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
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'NAV2 goal → {label} ({x:.2f}, {y:.2f})')
        self.get_logger().info('Planner: active')

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('NAV2 rechazó el goal.')
            self._publish_status('Goal rechazado por NAV2.')
            with self._fsm_lock:
                self._fsm.trigger('goal_failed')
            self._start_recovery()
            return

        self._current_goal_handle = goal_handle
        self.get_logger().info('Goal aceptado por NAV2 — esperando resultado...')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future) -> None:
        self._current_goal_handle = None
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('NAV2: Goal succeeded ✓')
            self._publish_status('Llegué al destino.')
            with self._fsm_lock:
                self._fsm.trigger('goal_succeeded')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('NAV2: Goal cancelado.')
            with self._fsm_lock:
                self._fsm.trigger('goal_cancelled')
        else:
            self.get_logger().warn(f'NAV2: Goal fallido (status={status}).')
            self._publish_status(f'No pude llegar al destino (status={status}).')
            with self._fsm_lock:
                self._fsm.trigger('goal_failed')
            self._start_recovery()

    # ── FSM callback ─────────────────────────────────────────────────────────

    def _on_state_change(self, old: State, event: str, new: State) -> None:
        self.get_logger().info(
            f'FSM: {old.value} ──{event}──→ {new.value}')
        self._publish_fsm_state(new)

    # ── Utilidades ────────────────────────────────────────────────────────────

    def _find_waypoint(self, target: str) -> tuple[str, dict] | None:
        t = self._normalize_location(target)
        if t in self._aliases:
            waypoint_name = self._aliases[t]
            waypoint = self._waypoints.get(waypoint_name)
            if waypoint is not None:
                return waypoint_name, waypoint

        for alias, waypoint_name in self._aliases.items():
            if alias and alias in t:
                waypoint = self._waypoints.get(waypoint_name)
                if waypoint is not None:
                    return waypoint_name, waypoint
        return None

    def _normalize_location(self, value: str) -> str:
        text = str(value or '').strip().lower().replace('_', ' ')
        text = ''.join(
            c for c in unicodedata.normalize('NFD', text)
            if unicodedata.category(c) != 'Mn'
        )
        return ' '.join(text.split()).replace(' ', '_')

    def _validate_waypoint(self, name: str, waypoint: dict) -> tuple[bool, str]:
        if not isinstance(waypoint, dict):
            return False, 'el waypoint no es un diccionario YAML'
        for field in ('x', 'y'):
            if field not in waypoint:
                return False, f'falta campo {field!r}'
            try:
                value = float(waypoint[field])
            except (TypeError, ValueError):
                return False, f'{field} no es numérico: {waypoint[field]!r}'
            if not math.isfinite(value):
                return False, f'{field} no es finito: {value!r}'
            if abs(value) > self._max_abs_coordinate:
                return (
                    False,
                    f'{field}={value:.2f} excede límite '
                    f'±{self._max_abs_coordinate:.1f} m',
                )
        try:
            yaw = float(waypoint.get('yaw', 0.0))
        except (TypeError, ValueError):
            return False, f'yaw no es numérico: {waypoint.get("yaw")!r}'
        if not math.isfinite(yaw):
            return False, f'yaw no es finito: {yaw!r}'
        if not name:
            return False, 'nombre de waypoint vacío'
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
