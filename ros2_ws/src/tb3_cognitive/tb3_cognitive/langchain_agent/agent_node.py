"""
langchain_agent/agent_node.py — Orquestador cognitivo Fase 2
-------------------------------------------------------------

Lee /cognitive/command (del NLP) y ejecuta la herramienta ROS2
correspondiente a través de LangChain tools.

Arquitectura:
  /cognitive/command  (tb3_msgs/CognitiveCommand)
       ↓
  CognitiveAgentNode
       ↓ routing por action
  ┌────────────────────────────────────────────┐
  │  navigate  → NAV2 /navigate_to_pose action │
  │  stop      → /cmd_vel Twist(0,0,0)         │
  │  search    → NAV2 goal (posición central)  │
  │  explore   → NAV2 goal (punto aleatorio)   │
  │  approach  → stub (Fase 4: YOLO)           │
  │  report    → log de estado                 │
  └────────────────────────────────────────────┘
       ↓
  /cognitive/status  (std_msgs/String)

Fase 3: el routing se reemplaza por AgentExecutor + LLM
        para razonamiento multi-step ("busca una botella Y acércate").

Topics:
  Sub:  /cognitive/command   (tb3_msgs/CognitiveCommand)
  Pub:  /cognitive/status    (std_msgs/String)
  Act:  /navigate_to_pose    (nav2_msgs/action/NavigateToPose)
  Pub:  /cmd_vel             (geometry_msgs/Twist)

Parámetros:
  waypoints_file  — ruta al YAML con coordenadas
  nav_timeout     — segundos para esperar servidor NAV2 (default 5.0)
"""

import os
import yaml
import random
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from tb3_msgs.msg import CognitiveCommand

from .tools import make_tools


class CognitiveAgentNode(Node):

    def __init__(self):
        super().__init__('cognitive_agent_node')

        # ── Parámetros ────────────────────────────────────────────────────────
        waypoints_file = self.declare_parameter(
            'waypoints_file', '').get_parameter_value().string_value
        self._nav_timeout = self.declare_parameter(
            'nav_timeout', 5.0).get_parameter_value().double_value

        # ── Waypoints ─────────────────────────────────────────────────────────
        self._waypoints = self._load_waypoints(waypoints_file)
        self.get_logger().info(
            f'Waypoints cargados: {list(self._waypoints.keys())}')

        # ── Publishers y clientes ROS2 ────────────────────────────────────────
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        self._cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self._status_pub = self.create_publisher(
            String, '/cognitive/status', 10)

        # ── LangChain tools ───────────────────────────────────────────────────
        # Closure sobre self → las tools tienen acceso a ROS2 directamente
        self._tools = make_tools(self)

        # ── Suscripción a comandos cognitivos ─────────────────────────────────
        self._sub = self.create_subscription(
            CognitiveCommand, '/cognitive/command',
            self._command_callback, 10)

        self.get_logger().info(
            'CognitiveAgentNode activo.\n'
            '  Sub: /cognitive/command\n'
            '  Pub: /cognitive/status\n'
            '  Act: /navigate_to_pose')

    # ── Carga de waypoints ────────────────────────────────────────────────────

    def _load_waypoints(self, filepath: str) -> dict:
        if not filepath or not os.path.exists(filepath):
            self.get_logger().warn(
                f'waypoints_file no existe: {filepath!r}\n'
                'Edita config/waypoints.yaml y pasa la ruta al launcher.')
            return {}
        with open(filepath) as f:
            data = yaml.safe_load(f)
        return data.get('waypoints', {})

    # ── Callback principal ────────────────────────────────────────────────────

    def _command_callback(self, msg: CognitiveCommand) -> None:
        self.get_logger().info(
            f'Recibido → action={msg.action!r}  '
            f'target={msg.target!r}  conf={msg.confidence:.2f}')

        tool = self._tools.get(msg.action)
        if tool is None:
            self.get_logger().warn(f'Acción no soportada: {msg.action!r}')
            return

        # Ejecuta en thread para no bloquear el executor de ROS2
        threading.Thread(
            target=self._run_tool,
            args=(tool, msg.target),
            daemon=True,
        ).start()

    def _run_tool(self, tool, target: str) -> None:
        try:
            result = tool.invoke({'target': target})
            self.get_logger().info(f'Resultado: {result}')
            status = String()
            status.data = result
            self._status_pub.publish(status)
        except Exception as exc:
            self.get_logger().error(f'Error en tool: {exc}')

    # ── Implementaciones ROS2 de cada acción ──────────────────────────────────

    def navigate_to_location(self, target: str) -> str:
        waypoint = self._find_waypoint(target)
        if waypoint is None:
            available = list(self._waypoints.keys())
            return (f'Waypoint desconocido: {target!r}. '
                    f'Disponibles: {available}')

        if not self._nav_client.wait_for_server(
                timeout_sec=self._nav_timeout):
            return ('NAV2 no responde. '
                    'Lanza navigation.launch.py antes del agente.')

        x = float(waypoint['x'])
        y = float(waypoint['y'])

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self._nav_client.send_goal_async(goal)
        self.get_logger().info(
            f'NAV2 goal → {target} ({x:.2f}, {y:.2f})')
        return f'Navegando a {target!r} ({x:.2f}, {y:.2f})'

    def stop_robot(self) -> str:
        twist = Twist()
        for _ in range(5):
            self._cmd_vel_pub.publish(twist)
        self.get_logger().info('Robot detenido')
        return 'Robot detenido'

    def search_for_object(self, target: str) -> str:
        # Fase 2: navega al origen y gira para buscar con los sensores
        # Fase 4 (YOLO): navega directamente hacia la detección visual
        if not self._nav_client.wait_for_server(
                timeout_sec=self._nav_timeout):
            return 'NAV2 no responde.'

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = 0.0
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.orientation.w = 1.0

        self._nav_client.send_goal_async(goal)
        self.get_logger().info(f'Buscando {target!r}: navegando a origen')
        return f'Buscando {target!r}: dirigiéndome al centro del mapa'

    def start_exploration(self, target: str = '') -> str:
        # Fase 2: navega a un punto aleatorio de exploración
        # Fase 3 (FSM): ciclo completo con estados EXPLORATION
        exploration_points = [
            (1.5,  1.5),
            (-1.5, 1.5),
            (1.5, -1.5),
            (-1.5, -1.5),
            (0.0,  2.0),
            (2.0,  0.0),
        ]
        x, y = random.choice(exploration_points)

        if not self._nav_client.wait_for_server(
                timeout_sec=self._nav_timeout):
            return 'NAV2 no responde.'

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0

        self._nav_client.send_goal_async(goal)
        self.get_logger().info(f'Explorando hacia ({x:.1f}, {y:.1f})')
        return f'Explorando: navegando a ({x:.1f}, {y:.1f})'

    def approach_target(self, target: str = '') -> str:
        # Stub — Fase 4 (YOLO) implementará approach visual
        return (f'Approach hacia {target!r}: '
                'disponible en Fase 4 con detección YOLO.')

    def report_status(self) -> str:
        nav_ok = self._nav_client.server_is_ready()
        wps = len(self._waypoints)
        return (f'Estado: NAV2={"OK" if nav_ok else "NO DISPONIBLE"}  '
                f'waypoints={wps}  nodo=activo')

    # ── Utilidades ────────────────────────────────────────────────────────────

    def _find_waypoint(self, target: str) -> dict | None:
        t = target.lower().strip()
        if t in self._waypoints:
            return self._waypoints[t]
        # Búsqueda parcial: "cocina" matchea "la cocina"
        for key, val in self._waypoints.items():
            if t in key or key in t:
                return val
        return None


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CognitiveAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
