"""
fsm/states.py — Máquina de estados finita del robot cognitivo

Estados:
  IDLE        — esperando comando
  NAVIGATING  — ejecutando NavigateToPose hacia un waypoint
  EXPLORING   — navegando a un punto aleatorio de exploración
  SEARCHING   — buscando un objeto (navega al origen + gira)
  APPROACHING — acercándose a un objetivo detectado (Fase 4)
  STOPPED     — parada explícita/cancelación de seguridad
  REPORTING   — publicando estado del sistema
  ERROR       — goal fallido
  RECOVERY    — limpieza/reset de navegación tras fallo

Eventos válidos:
  navigate, explore, search, approach, report  — inician una acción desde IDLE
  goal_succeeded, goal_failed, goal_cancelled  — resultado de NAV2
  stop                                         — detiene cualquier acción activa
  recovery_start/recovery_done                 — recuperación tras ERROR
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
    STOPPED     = 'STOPPED'
    REPORTING   = 'REPORTING'
    ERROR       = 'ERROR'
    RECOVERY    = 'RECOVERY'


_TRANSITIONS: Dict[Tuple[State, str], State] = {
    # Desde IDLE → iniciar acción
    (State.IDLE, 'navigate'):               State.NAVIGATING,
    (State.IDLE, 'explore'):                State.EXPLORING,
    (State.IDLE, 'search'):                 State.SEARCHING,
    (State.IDLE, 'approach'):               State.APPROACHING,
    (State.IDLE, 'report'):                 State.REPORTING,
    # Resultado de NAV2
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
    (State.REPORTING,   'done'):            State.IDLE,
    # Stop desde cualquier estado activo
    (State.NAVIGATING,  'stop'):            State.STOPPED,
    (State.EXPLORING,   'stop'):            State.STOPPED,
    (State.SEARCHING,   'stop'):            State.STOPPED,
    (State.APPROACHING, 'stop'):            State.STOPPED,
    (State.RECOVERY,    'stop'):            State.STOPPED,
    # Estados terminales/controlados
    (State.STOPPED, 'reset'):               State.IDLE,
    (State.STOPPED, 'stop'):                State.STOPPED,
    (State.ERROR, 'reset'):                 State.IDLE,
    (State.ERROR, 'stop'):                  State.STOPPED,
    (State.ERROR, 'recovery_start'):        State.RECOVERY,
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
        return self._state not in (State.IDLE, State.ERROR, State.STOPPED)

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
