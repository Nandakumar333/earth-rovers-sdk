"""Central command safety arbiter executing multi-gate verification."""

import time
from typing import Optional
from autonomy.config import AutonomyConfig
from autonomy.contracts.commands import MotionCommand
from autonomy.contracts.perception import CameraHealthScore
from autonomy.contracts.state import RoverState
from autonomy.observability.events import EventManager
from autonomy.observability.metrics import AutonomyMetrics
from autonomy.safety.contracts import DecisionStatus, RolloverRiskLevel, SafetyDecision
from autonomy.safety.health_gate import SensorHealthGate
from autonomy.safety.recovery import RecoveryStateMachine
from autonomy.safety.rollover import RolloverEstimator
from autonomy.safety.stuck import StuckDetector
from autonomy.safety.watchdogs import NetworkWatchdog, TTLWatchdog


class SafetyArbiter:
    """Evaluates candidate motion commands against active edge safety constraints."""

    def __init__(
        self,
        config: AutonomyConfig,
        event_manager: Optional[EventManager] = None,
        metrics: Optional[AutonomyMetrics] = None,
    ):
        self.config = config
        self.events = event_manager or EventManager()
        self.metrics = metrics or AutonomyMetrics()

        self.ttl_watchdog = TTLWatchdog(max_ttl_ms=config.safety.command_ttl_ms)
        self.network_watchdog = NetworkWatchdog(
            disconnect_timeout_s=config.network.disconnect_timeout_s
        )
        self.rollover = RolloverEstimator(config=config.safety)
        self.stuck = StuckDetector(config=config.safety)
        self.health_gate = SensorHealthGate(config=config.sensors)
        self.recovery = RecoveryStateMachine()

    def arbitrate(
        self,
        command: MotionCommand,
        rover_state: RoverState,
        camera_health: Optional[CameraHealthScore] = None,
        current_time: Optional[float] = None,
    ) -> SafetyDecision:
        """Run safety gate hierarchy: Recovery > TTL > Network > Rollover > Stuck > Health."""
        now = time.time() if current_time is None else current_time

        # 1. Recovery Mode Takeover
        if self.recovery.is_active:
            rec_cmd = self.recovery.step(current_time=now)
            if rec_cmd:
                return SafetyDecision(
                    status=DecisionStatus.SCALED,
                    command=rec_cmd,
                    reason=f"Recovery maneuver in progress (state={self.recovery.state.value})",
                    trigger="RECOVERY_ACTIVE",
                )

        # 2. Command TTL Gate
        safe, ttl_dec = self.ttl_watchdog.evaluate(command, current_time=now)
        if not safe and ttl_dec:
            self._record_veto(ttl_dec)
            return ttl_dec

        # 3. Network / Telemetry Staleness Gate
        safe, net_dec = self.network_watchdog.evaluate(rover_state)
        if not safe and net_dec:
            self._record_veto(net_dec)
            return net_dec

        # 4. Rollover Risk Gate
        risk_level, compound_deg, roll_reason = self.rollover.evaluate(rover_state)
        if risk_level == RolloverRiskLevel.EMERGENCY:
            dec = SafetyDecision(
                status=DecisionStatus.EMERGENCY_STOP,
                command=MotionCommand.create_stop(priority=0, source="arbiter_rollover"),
                reason=roll_reason,
                trigger="ROLLOVER_EMERGENCY",
                telemetry_snapshot={"compound_angle_deg": compound_deg},
            )
            self._record_veto(dec)
            return dec

        if risk_level == RolloverRiskLevel.HIGH_RISK:
            dec = SafetyDecision(
                status=DecisionStatus.VETOED,
                command=MotionCommand.create_stop(priority=1, source="arbiter_rollover"),
                reason=roll_reason,
                trigger="ROLLOVER_HIGH_RISK",
                telemetry_snapshot={"compound_angle_deg": compound_deg},
            )
            self._record_veto(dec)
            return dec

        candidate_cmd = command
        status = DecisionStatus.ALLOWED
        reason = "All safety gates passed"
        trigger = None

        if risk_level == RolloverRiskLevel.CAUTION:
            # Derate linear velocity to 60%
            candidate_cmd = candidate_cmd.model_copy(
                update={"linear": candidate_cmd.linear * 0.6}
            )
            status = DecisionStatus.SCALED
            reason = roll_reason
            trigger = "ROLLOVER_CAUTION"

        # 5. Stuck & Traction Gate
        is_stuck, slip, stuck_reason = self.stuck.evaluate(candidate_cmd, rover_state, current_time=now)
        if is_stuck:
            self.recovery.trigger_stuck(reason=stuck_reason)
            dec = SafetyDecision(
                status=DecisionStatus.VETOED,
                command=MotionCommand.create_stop(priority=1, source="arbiter_stuck"),
                reason=stuck_reason,
                trigger="STUCK_DETECTED",
                telemetry_snapshot={"slip_ratio": slip},
            )
            self._record_veto(dec)
            return dec

        # 6. Perception & Camera Health Gate
        safe, health_dec = self.health_gate.evaluate(candidate_cmd, camera_health)
        if not safe and health_dec:
            self._record_veto(health_dec)
            return health_dec

        if health_dec and health_dec.status == DecisionStatus.SCALED:
            candidate_cmd = health_dec.command
            status = DecisionStatus.SCALED
            reason = health_dec.reason
            trigger = health_dec.trigger

        # 7. Command Passes All Gates
        self.metrics.record_command(latency_ms=0.0, vetoed=False)
        return SafetyDecision(
            status=status,
            command=candidate_cmd,
            reason=reason,
            trigger=trigger,
        )

    def _record_veto(self, decision: SafetyDecision) -> None:
        """Log safety veto to metrics and events."""
        self.metrics.record_command(latency_ms=0.0, vetoed=True)
        if decision.status in (DecisionStatus.VETOED, DecisionStatus.EMERGENCY_STOP):
            self.metrics.record_safety_stop()
        self.events.record(
            event_type="SAFETY_VETO",
            reason=decision.reason,
            details={
                "trigger": decision.trigger,
                "status": decision.status.value,
                "telemetry": decision.telemetry_snapshot,
            },
        )
