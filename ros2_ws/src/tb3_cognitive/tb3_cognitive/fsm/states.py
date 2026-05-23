"""
fsm/states.py — Máquina de estados finita del robot cognitivo

Estados:
  IDLE        — esperando comando
  NAVIGATING  — ejecutando NavigateToPose hacia un waypoint
  EXPLORING   — patrullando waypoints secuencialmente
  SEARCHING   — recorriendo habitaciones buscando un objeto
  APPROACHING — acercándose a objetivo detectado (Fase 5)
  GRASPING    — ejecutando trayectoria de agarre con el brazo (Fase 6)
  STOPPING    — cancelación en curso (cancel_goal_async enviado)
  STOPPED     — parado, listo para siguiente comando
  REPORTING   — publicando estado del sistema
  ERROR       — goal fallido, esperando recovery
  RECOVERY    — limpiando costmaps + reset de navegación
  UNKNOWN     — comando con confianza insuficiente (transiente)

Eventos válidos:
  navigate, explore, search, approach, report  — inician acción desde IDLE
  goal_succeeded, goal_failed, goal_cancelled  — resultado de NAV2
  grasp                                        — APPROACHING completado → iniciar agarre
  grasp_done, grasp_failed                     — resultado del brazo
  stop                                         — cancela acción activa
  recovery_start / recovery_done               — ciclo recovery tras ERROR
  reset                                        — salida manual de ERROR/STOPPED
  done                                         — termina REPORTING
"""

from enum import Enum
from typing import Callable, Dict, Optional, Tuple


class State(str, Enum):
    IDLE        = 'IDLE'
    NAVIGATING  = 'NAVIGATING'
    EXPLORING   = 'EXPLORING'
    SEARCHING   = 'SEARCHING'
    APPROACHING = 'APPROACHING'
    GRASPING    = 'GRASPING'
    STOPPING    = 'STOPPING'
    STOPPED     = 'STOPPED'
    REPORTING   = 'REPORTING'
    ERROR       = 'ERROR'
    RECOVERY    = 'RECOVERY'
    UNKNOWN     = 'UNKNOWN'


_TRANSITIONS: Dict[Tuple[State, str], State] = {
    # Desde IDLE → iniciar acción
    (State.IDLE, 'navigate'):               State.NAVIGATING,
    (State.IDLE, 'explore'):                State.EXPLORING,
    (State.IDLE, 'search'):                 State.SEARCHING,
    (State.IDLE, 'approach'):               State.APPROACHING,
    (State.IDLE, 'report'):                 State.REPORTING,
    # Resultado de NAV2 — estados activos
    (State.NAVIGATING,  'goal_succeeded'):  State.IDLE,
    (State.NAVIGATING,  'goal_failed'):     State.ERROR,
    (State.NAVIGATING,  'goal_cancelled'):  State.STOPPED,
    (State.EXPLORING,   'goal_succeeded'):  State.IDLE,
    (State.EXPLORING,   'goal_failed'):     State.ERROR,
    (State.EXPLORING,   'goal_cancelled'):  State.STOPPED,
    (State.SEARCHING,   'goal_succeeded'):  State.IDLE,
    (State.SEARCHING,   'goal_failed'):     State.ERROR,
    (State.SEARCHING,   'goal_cancelled'):  State.STOPPED,
    (State.APPROACHING, 'goal_succeeded'):  State.IDLE,
    (State.APPROACHING, 'goal_failed'):     State.ERROR,
    (State.APPROACHING, 'goal_cancelled'):  State.STOPPED,
    (State.APPROACHING, 'grasp'):           State.GRASPING,
    # Resultado del brazo (Fase 6)
    (State.GRASPING, 'grasp_done'):         State.IDLE,
    (State.GRASPING, 'grasp_failed'):       State.ERROR,
    (State.GRASPING, 'stop'):               State.IDLE,
    # Detección visual YOLO durante búsqueda → acercamiento
    (State.SEARCHING, 'target_found'):      State.APPROACHING,
    (State.REPORTING, 'done'):              State.IDLE,
    # Stop → STOPPING (cancelación en curso)
    (State.NAVIGATING,  'stop'):            State.STOPPING,
    (State.EXPLORING,   'stop'):            State.STOPPING,
    (State.SEARCHING,   'stop'):            State.STOPPING,
    (State.APPROACHING, 'stop'):            State.STOPPING,
    (State.RECOVERY,    'stop'):            State.STOPPING,
    # STOPPING → STOPPED cuando el goal responde (cualquier resultado)
    (State.STOPPING, 'goal_cancelled'):     State.STOPPED,
    (State.STOPPING, 'goal_succeeded'):     State.STOPPED,
    (State.STOPPING, 'goal_failed'):        State.STOPPED,
    # UNKNOWN (transiente — siempre vuelve a IDLE)
    (State.IDLE,    'unknown'):             State.UNKNOWN,
    (State.UNKNOWN, 'reset'):               State.IDLE,
    (State.UNKNOWN, 'stop'):                State.IDLE,
    # STOPPED / ERROR → IDLE
    (State.STOPPED, 'reset'):               State.IDLE,
    (State.STOPPED, 'stop'):                State.STOPPED,
    (State.ERROR,   'reset'):               State.IDLE,
    (State.ERROR,   'stop'):                State.STOPPED,
    (State.ERROR,   'recovery_start'):      State.RECOVERY,
    (State.RECOVERY, 'recovery_done'):      State.IDLE,
    (State.RECOVERY, 'recovery_failed'):    State.ERROR,
}


class RobotFSM:

    def __init__(self, on_transition: Optional[Callable] = None):
        self._state = State.IDLE
        # on_transition(old_state, event, new_state)
        self._on_transition = on_transition

    @property
    def state(self) -> State:
        return self._state

    def is_busy(self) -> bool:
        return self._state not in (State.IDLE, State.ERROR, State.STOPPED, State.STOPPING)

    def can_trigger(self, event: str) -> bool:
        return (self._state, event) in _TRANSITIONS

    def trigger(self, event: str) -> bool:
        """Intenta disparar un evento. Devuelve True si la transición ocurrió."""
        key = (self._state, event)
        if key not in _TRANSITIONS:
            return False
        old = self._state
        self._state = _TRANSITIONS[key]
        if self._on_transition:
            self._on_transition(old, event, self._state)
        return True
