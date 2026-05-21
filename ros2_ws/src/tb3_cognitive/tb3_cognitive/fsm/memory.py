"""
fsm/memory.py — Memoria cognitiva de sesión

Almacena el historial reciente de comandos, navegaciones y resultados.
Prepara la capa de contexto para razonamiento futuro (LangChain, Fase 5).

No persiste entre reinicios del nodo — es memoria de trabajo (working memory).
"""

import time
from dataclasses import dataclass, field
from typing import Optional, Tuple


@dataclass
class CognitiveMemory:

    # ── Último comando recibido ───────────────────────────────────────────────
    last_action:        Optional[str]   = None
    last_target:        Optional[str]   = None
    last_confidence:    float           = 0.0
    last_command_time:  float           = 0.0

    # ── Última navegación ejecutada ───────────────────────────────────────────
    last_waypoint:      Optional[str]   = None
    last_coordinates:   Optional[Tuple[float, float]] = None
    last_nav_time:      float           = 0.0

    # ── Último resultado ──────────────────────────────────────────────────────
    last_success_location: Optional[str] = None
    last_success_time:     float         = 0.0
    last_failure_reason:   Optional[str] = None
    last_failure_time:     float         = 0.0

    # ── Contadores acumulados ─────────────────────────────────────────────────
    command_count:  int = 0
    success_count:  int = 0
    failure_count:  int = 0
    recovery_count: int = 0

    def record_command(self, action: str, target: str, confidence: float) -> None:
        self.last_action     = action
        self.last_target     = target
        self.last_confidence = confidence
        self.last_command_time = time.time()
        self.command_count  += 1

    def record_navigation(self, waypoint: str, x: float, y: float) -> None:
        self.last_waypoint    = waypoint
        self.last_coordinates = (x, y)
        self.last_nav_time    = time.time()

    def record_success(self, location: str) -> None:
        self.last_success_location = location
        self.last_success_time     = time.time()
        self.success_count        += 1

    def record_failure(self, reason: str) -> None:
        self.last_failure_reason = reason
        self.last_failure_time   = time.time()
        self.failure_count      += 1

    def record_recovery(self) -> None:
        self.recovery_count += 1

    def summary(self) -> str:
        lines = [
            f'Comandos: {self.command_count}  '
            f'Éxitos: {self.success_count}  '
            f'Fallos: {self.failure_count}  '
            f'Recoveries: {self.recovery_count}',
        ]
        if self.last_action:
            age = time.time() - self.last_command_time
            lines.append(
                f'Último cmd: {self.last_action!r} → {self.last_target!r} '
                f'(conf={self.last_confidence:.2f}, hace {age:.0f}s)')
        if self.last_waypoint:
            x, y = self.last_coordinates or (0, 0)
            lines.append(
                f'Última nav: {self.last_waypoint} ({x:.2f}, {y:.2f})')
        if self.last_success_location:
            age = time.time() - self.last_success_time
            lines.append(
                f'Último éxito: {self.last_success_location} hace {age:.0f}s')
        if self.last_failure_reason:
            age = time.time() - self.last_failure_time
            lines.append(
                f'Último fallo: {self.last_failure_reason!r} hace {age:.0f}s')
        return ' | '.join(lines)
