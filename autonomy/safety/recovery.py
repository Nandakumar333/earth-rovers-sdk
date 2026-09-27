"""Recovery state machine for stuck condition mitigation."""

from enum import Enum
import time
from typing import Optional
from autonomy.contracts.commands import MotionCommand


class RecoveryState(str, Enum):
    """Sub-states of the stuck recovery cycle."""

    IDLE = "IDLE"
    STOP = "STOP"
    VERIFY = "VERIFY"
    EXECUTING_MANEUVER = "EXECUTING_MANEUVER"
    VERIFY_PROGRESS = "VERIFY_PROGRESS"
    ESCALATED = "ESCALATED"


class RecoveryStateMachine:
    """Orchestrates short, bounded, safety-arbitrated recovery actions."""

    def __init__(self, max_attempts: int = 3):
        self.max_attempts = max_attempts
        self.state: RecoveryState = RecoveryState.IDLE
        self.attempt_count: int = 0
        self._maneuver_start_time: float = 0.0
        self._current_command: Optional[MotionCommand] = None

    @property
    def is_active(self) -> bool:
        """Whether a recovery routine is currently in progress."""
        return self.state not in (RecoveryState.IDLE, RecoveryState.ESCALATED)

    def trigger_stuck(self, reason: str = "") -> None:
        """Initiate recovery sequence upon detected stuck condition."""
        if self.state == RecoveryState.IDLE:
            self.state = RecoveryState.STOP
            self.attempt_count = 1
        elif self.state == RecoveryState.VERIFY_PROGRESS:
            self.attempt_count += 1
            if self.attempt_count > self.max_attempts:
                self.state = RecoveryState.ESCALATED
            else:
                self.state = RecoveryState.STOP

    def step(self, current_time: Optional[float] = None) -> Optional[MotionCommand]:
        """Advance recovery progression and return immediate recovery motion command if active."""
        now = time.time() if current_time is None else current_time

        if self.state == RecoveryState.IDLE:
            return None

        if self.state == RecoveryState.STOP:
            # First deliver zero stop to let rover settle
            self.state = RecoveryState.VERIFY
            return MotionCommand.create_stop(priority=5, source="recovery_stop", ttl_s=0.5)

        if self.state == RecoveryState.VERIFY:
            # Select maneuver: alternate between gentle reverse pulse and pivot
            now = time.time() if current_time is None else current_time
            self._maneuver_start_time = now
            self.state = RecoveryState.EXECUTING_MANEUVER

            if self.attempt_count % 2 == 1:
                # Short reverse pulse (-0.25 linear for 400ms)
                cmd = MotionCommand(
                    linear=-0.25,
                    angular=0.0,
                    valid_from=now,
                    expires_at=now + 0.4,
                    source="recovery_pulse",
                    priority=5,
                )
            else:
                # Short gentle pivot (angular 0.3 for 400ms)
                cmd = MotionCommand(
                    linear=-0.1,
                    angular=0.3,
                    valid_from=now,
                    expires_at=now + 0.4,
                    source="recovery_pivot",
                    priority=5,
                )
            self._current_command = cmd
            return cmd

        if self.state == RecoveryState.EXECUTING_MANEUVER:
            elapsed = now - self._maneuver_start_time
            if elapsed >= 0.4:
                # Maneuver complete, enter verify progress
                self.state = RecoveryState.VERIFY_PROGRESS
                return MotionCommand.create_stop(priority=5, source="recovery_settle", ttl_s=0.3)
            return self._current_command

        if self.state == RecoveryState.VERIFY_PROGRESS:
            return None

        if self.state == RecoveryState.ESCALATED:
            # Max recovery attempts exhausted: stay stopped
            return MotionCommand.create_stop(priority=1, source="recovery_escalated", ttl_s=1.0)

        return None

    def resolve(self) -> None:
        """Traction restored successfully: reset recovery machine."""
        self.state = RecoveryState.IDLE
        self.attempt_count = 0
        self._current_command = None

    def reset(self) -> None:
        """Reset state machine entirely."""
        self.state = RecoveryState.IDLE
        self.attempt_count = 0
        self._current_command = None
