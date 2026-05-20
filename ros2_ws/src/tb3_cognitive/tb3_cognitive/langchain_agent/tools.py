"""
langchain_agent/tools.py — Herramientas ROS2 como LangChain Tools
------------------------------------------------------------------

Cada herramienta es una función que llama a la API ROS2 correspondiente.
Las funciones cierran sobre el nodo ROS2 (closure pattern) para tener
acceso a action clients, publishers y parámetros.

Fase 2: tool routing directo (el NLP ya clasifica la intención).
Fase 3: estas mismas tools se conectarán a un AgentExecutor con LLM
        para razonamiento multi-step y descomposición de tareas.

Tools disponibles:
  navigate  → NavigateToPose action → NAV2
  stop      → /cmd_vel Twist cero
  search    → navega al centro y gira (Fase 4: YOLO guiará)
  explore   → navega a punto aleatorio de exploración
  approach  → stub (Fase 4: visión guiará el approach)
  report    → devuelve estado actual del sistema
"""

from langchain_core.tools import StructuredTool
from pydantic import BaseModel, Field


# ── Schemas de entrada ────────────────────────────────────────────────────────

class TargetInput(BaseModel):
    target: str = Field(default='', description='Destino, objeto o nombre del target')


# ── Factory de tools ──────────────────────────────────────────────────────────

def make_tools(node) -> dict:
    """
    Crea las LangChain tools con acceso al nodo ROS2 via closure.
    El argumento 'node' es una instancia de CognitiveAgentNode.

    Retorna dict {action_name: StructuredTool}.
    """

    # ── navigate ──────────────────────────────────────────────────────────────
    def _navigate(target: str = '') -> str:
        return node.navigate_to_location(target)

    navigate_tool = StructuredTool.from_function(
        func=_navigate,
        name='navigate',
        description='Navega el robot a una habitación o ubicación específica del mapa.',
        args_schema=TargetInput,
    )

    # ── stop ──────────────────────────────────────────────────────────────────
    def _stop(target: str = '') -> str:
        return node.stop_robot()

    stop_tool = StructuredTool.from_function(
        func=_stop,
        name='stop',
        description='Detiene todo el movimiento del robot inmediatamente.',
        args_schema=TargetInput,
    )

    # ── search ────────────────────────────────────────────────────────────────
    def _search(target: str = '') -> str:
        return node.search_for_object(target)

    search_tool = StructuredTool.from_function(
        func=_search,
        name='search',
        description='Busca un objeto específico navegando a su última posición conocida.',
        args_schema=TargetInput,
    )

    # ── explore ───────────────────────────────────────────────────────────────
    def _explore(target: str = '') -> str:
        return node.start_exploration(target)

    explore_tool = StructuredTool.from_function(
        func=_explore,
        name='explore',
        description='Explora el entorno navegando a puntos aleatorios del mapa.',
        args_schema=TargetInput,
    )

    # ── approach ──────────────────────────────────────────────────────────────
    def _approach(target: str = '') -> str:
        return node.approach_target(target)

    approach_tool = StructuredTool.from_function(
        func=_approach,
        name='approach',
        description='Se acerca a un objetivo cercano detectado visualmente.',
        args_schema=TargetInput,
    )

    # ── report ────────────────────────────────────────────────────────────────
    def _report(target: str = '') -> str:
        return node.report_status()

    report_tool = StructuredTool.from_function(
        func=_report,
        name='report',
        description='Informa sobre el estado actual del robot y su entorno.',
        args_schema=TargetInput,
    )

    return {
        'navigate': navigate_tool,
        'stop':     stop_tool,
        'search':   search_tool,
        'explore':  explore_tool,
        'approach': approach_tool,
        'report':   report_tool,
    }
