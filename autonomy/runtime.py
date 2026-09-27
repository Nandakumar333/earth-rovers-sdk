"""Autonomy runtime lifecycle and explicit state machine manager."""

import asyncio
import logging
from typing import Dict, Optional, Set
from autonomy.config import AutonomyConfig, load_autonomy_config
from autonomy.contracts.commands import MotionCommand
from autonomy.contracts.perception import CameraHealthScore
from autonomy.contracts.state import AutonomyState, RoverState
from autonomy.observability.events import EventManager
from autonomy.observability.metrics import AutonomyMetrics
from autonomy.safety.arbiter import SafetyArbiter
from autonomy.safety.contracts import DecisionStatus, SafetyDecision

logger = logging.getLogger("autonomy.runtime")


class StateTransitionError(ValueError):
    """Raised when an illegal state machine transition is attempted."""


class AutonomyRuntime:
    """Manages autonomy state transitions, background loops, and lifecycle."""

    # Map of allowed transitions: current_state -> set of valid target_states
    VALID_TRANSITIONS: Dict[AutonomyState, Set[AutonomyState]] = {
        AutonomyState.DISABLED: {
            AutonomyState.INITIALIZING,
        },
        AutonomyState.INITIALIZING: {
            AutonomyState.READY,
            AutonomyState.DISABLED,
            AutonomyState.EMERGENCY_STOP,
        },
        AutonomyState.READY: {
            AutonomyState.AUTONOMOUS,
            AutonomyState.STOP,
            AutonomyState.DISABLED,
            AutonomyState.MANUAL_ONLY,
            AutonomyState.EMERGENCY_STOP,
        },
        AutonomyState.AUTONOMOUS: {
            AutonomyState.READY,
            AutonomyState.DEFERRED,
            AutonomyState.STOP,
            AutonomyState.MANUAL_ONLY,
            AutonomyState.EMERGENCY_STOP,
        },
        AutonomyState.DEFERRED: {
            AutonomyState.AUTONOMOUS,
            AutonomyState.STOP,
            AutonomyState.MANUAL_ONLY,
            AutonomyState.EMERGENCY_STOP,
        },
        AutonomyState.STOP: {
            AutonomyState.READY,
            AutonomyState.RECOVERY,
            AutonomyState.MANUAL_ONLY,
            AutonomyState.EMERGENCY_STOP,
        },
        AutonomyState.RECOVERY: {
            AutonomyState.AUTONOMOUS,
            AutonomyState.READY,
            AutonomyState.STOP,
            AutonomyState.MANUAL_ONLY,
            AutonomyState.EMERGENCY_STOP,
        },
        AutonomyState.MANUAL_ONLY: {
            AutonomyState.READY,
            AutonomyState.DISABLED,
            AutonomyState.EMERGENCY_STOP,
        },
        AutonomyState.EMERGENCY_STOP: {
            AutonomyState.MANUAL_ONLY,
            AutonomyState.DISABLED,
        },
    }

    def __init__(
        self,
        config: Optional[AutonomyConfig] = None,
        event_manager: Optional[EventManager] = None,
        metrics: Optional[AutonomyMetrics] = None,
    ):
        self.config = config or load_autonomy_config()
        self.events = event_manager or EventManager()
        self.metrics = metrics or AutonomyMetrics()
        self.arbiter = SafetyArbiter(
            config=self.config,
            event_manager=self.events,
            metrics=self.metrics,
        )

        self._state: AutonomyState = (
            AutonomyState.DISABLED if not self.config.enabled else AutonomyState.DISABLED
        )
        self._lock: Optional[asyncio.Lock] = None
        self._lock_loop = None
        self._running = False
        self._tasks: Set[asyncio.Task] = set()

    @property
    def state(self) -> AutonomyState:
        """Current autonomy state."""
        return self._state

    @property
    def is_autonomous(self) -> bool:
        """Whether rover is currently navigating autonomously."""
        return self._state == AutonomyState.AUTONOMOUS

    @property
    def is_active(self) -> bool:
        """Whether autonomy runtime is running and healthy."""
        return self._state in (AutonomyState.READY, AutonomyState.AUTONOMOUS, AutonomyState.DEFERRED)

    def _get_lock(self) -> asyncio.Lock:
        running_loop = asyncio.get_running_loop()
        if self._lock is None or self._lock_loop is not running_loop:
            self._lock = asyncio.Lock()
            self._lock_loop = running_loop
        return self._lock

    def transition_to(
        self, target_state: AutonomyState, reason: str = "", details: Optional[dict] = None
    ) -> bool:
        """Synchronously request a valid state transition."""
        if target_state == self._state:
            return True

        # Any state can transition to EMERGENCY_STOP
        if target_state == AutonomyState.EMERGENCY_STOP:
            allowed = True
        else:
            allowed = target_state in self.VALID_TRANSITIONS.get(self._state, set())

        if not allowed:
            err_msg = f"Illegal transition: {self._state.value} -> {target_state.value}"
            logger.warning("%s (reason: %s)", err_msg, reason)
            raise StateTransitionError(err_msg)

        old_state = self._state
        self._state = target_state

        self.metrics.record_transition()
        if target_state in (AutonomyState.STOP, AutonomyState.EMERGENCY_STOP):
            self.metrics.record_safety_stop()

        self.events.record(
            event_type="STATE_TRANSITION",
            from_state=old_state.value,
            to_state=target_state.value,
            reason=reason,
            details=details,
        )
        return True

    async def start(self) -> None:
        """Initialize and start autonomy runtime."""
        if self._running:
            return

        self._running = True
        self.transition_to(AutonomyState.INITIALIZING, reason="Autonomy runtime starting")

        # Basic health self-check
        # (Sensor probes, state estimators, and monitors will register here in subsequent epics)
        self.transition_to(AutonomyState.READY, reason="Autonomy subsystems initialized successfully")
        logger.info("Autonomy runtime started successfully in state: %s", self._state.value)

    async def stop(self) -> None:
        """Gracefully stop autonomy runtime."""
        if not self._running:
            return

        self._running = False
        if self._state != AutonomyState.DISABLED:
            # Transition to STOP first if active
            if self._state in (AutonomyState.AUTONOMOUS, AutonomyState.DEFERRED, AutonomyState.RECOVERY):
                self.transition_to(AutonomyState.STOP, reason="Autonomy runtime shutdown requested")

            # Cancel running background tasks
            for task in list(self._tasks):
                if not task.done():
                    task.cancel()
            self._tasks.clear()

            self.transition_to(AutonomyState.DISABLED, reason="Autonomy runtime stopped")
        logger.info("Autonomy runtime stopped")

    def emergency_stop(self, reason: str = "Emergency stop invoked") -> None:
        """Immediate transition to EMERGENCY_STOP."""
        self.transition_to(AutonomyState.EMERGENCY_STOP, reason=reason)

    def reset_emergency(self, reason: str = "Operator cleared emergency stop") -> bool:
        """Reset from EMERGENCY_STOP to MANUAL_ONLY."""
        if self._state != AutonomyState.EMERGENCY_STOP:
            return False
        return self.transition_to(AutonomyState.MANUAL_ONLY, reason=reason)

    def arbitrate_command(
        self,
        command: MotionCommand,
        rover_state: RoverState,
        camera_health: Optional[CameraHealthScore] = None,
        current_time: Optional[float] = None,
    ) -> SafetyDecision:
        """Arbitrate a candidate command through the edge safety engine."""
        decision = self.arbiter.arbitrate(
            command=command,
            rover_state=rover_state,
            camera_health=camera_health,
            current_time=current_time,
        )

        # Trigger runtime state transition on emergency or veto
        if decision.status == DecisionStatus.EMERGENCY_STOP:
            if self._state != AutonomyState.EMERGENCY_STOP:
                self.transition_to(AutonomyState.EMERGENCY_STOP, reason=decision.reason)
        elif decision.status == DecisionStatus.VETOED and self._state == AutonomyState.AUTONOMOUS:
            self.transition_to(AutonomyState.STOP, reason=decision.reason)

        return decision

    def status(self) -> dict:
        """Detailed status snapshot of the autonomy subsystem."""
        return {
            "enabled": self.config.enabled,
            "state": self._state.value,
            "mode": self.config.mode,
            "is_autonomous": self.is_autonomous,
            "safety": {
                "recovery_state": self.arbiter.recovery.state.value,
            },
            "metrics": self.metrics.snapshot(),
            "recent_events": [e.model_dump() for e in self.events.get_events(limit=10)],
        }
