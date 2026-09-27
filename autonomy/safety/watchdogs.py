"""Watchdogs for command freshness and network telemetry latency."""

import time
from typing import Optional, Tuple
from autonomy.contracts.commands import MotionCommand
from autonomy.contracts.state import RoverState
from autonomy.safety.contracts import DecisionStatus, SafetyDecision


class TTLWatchdog:
    """Validates that candidate motion commands have not exceeded their TTL."""

    def __init__(self, max_ttl_ms: float = 500.0):
        self.max_ttl_ms = max_ttl_ms

    def evaluate(
        self, command: MotionCommand, current_time: Optional[float] = None
    ) -> Tuple[bool, Optional[SafetyDecision]]:
        """Return (is_safe, decision). If unsafe, decision contains zero-stop."""
        now = time.time() if current_time is None else current_time

        # Check if already expired
        if command.is_expired(now):
            return False, SafetyDecision(
                status=DecisionStatus.VETOED,
                command=MotionCommand.create_stop(priority=1, source="ttl_watchdog"),
                reason=f"Command expired: current_time={now:.3f} > expires_at={command.expires_at:.3f}",
                trigger="TTL_EXPIRED",
            )

        # Check if duration exceeds maximum allowable TTL
        command_ttl_s = command.expires_at - command.valid_from
        if command_ttl_s * 1000.0 > self.max_ttl_ms * 1.5:
            # Over-long TTL command; clamped/rejected to prevent runaway
            return False, SafetyDecision(
                status=DecisionStatus.VETOED,
                command=MotionCommand.create_stop(priority=1, source="ttl_watchdog"),
                reason=f"Command TTL {command_ttl_s * 1000.0:.1f}ms exceeds max limit {self.max_ttl_ms:.1f}ms",
                trigger="EXCESSIVE_TTL",
            )

        return True, None


class NetworkWatchdog:
    """Monitors telemetry staleness and connection dropouts."""

    def __init__(self, disconnect_timeout_s: float = 3.0):
        self.disconnect_timeout_s = disconnect_timeout_s

    def evaluate(
        self, rover_state: RoverState
    ) -> Tuple[bool, Optional[SafetyDecision]]:
        """Return (is_safe, decision). If telemetry is stale, trigger safe stop."""
        telemetry_age_s = rover_state.telemetry_age_ms / 1000.0
        if telemetry_age_s > self.disconnect_timeout_s:
            return False, SafetyDecision(
                status=DecisionStatus.VETOED,
                command=MotionCommand.create_stop(priority=2, source="network_watchdog"),
                reason=f"Telemetry lost: age {telemetry_age_s:.2f}s exceeds timeout {self.disconnect_timeout_s:.2f}s",
                trigger="NETWORK_DISCONNECTED",
                telemetry_snapshot={"telemetry_age_s": telemetry_age_s},
            )

        return True, None
