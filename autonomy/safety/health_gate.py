"""Sensor quality and camera degradation gating."""

from typing import Optional, Tuple
from autonomy.config import SensorsConfig
from autonomy.contracts.commands import MotionCommand
from autonomy.contracts.perception import CameraHealthScore
from autonomy.safety.contracts import DecisionStatus, SafetyDecision


class SensorHealthGate:
    """Modulates commanded motion based on perception certainty and camera degradation."""

    def __init__(self, config: SensorsConfig):
        self.config = config
        self.min_health = config.min_camera_health

    def evaluate(
        self, command: MotionCommand, camera_health: Optional[CameraHealthScore]
    ) -> Tuple[bool, Optional[SafetyDecision]]:
        """Evaluate command against visual perception health."""
        if camera_health is None:
            # No camera telemetry yet: proceed with baseline caution
            return True, None

        if not camera_health.is_usable or camera_health.score < 0.25:
            # Unusable camera: safe stop
            return False, SafetyDecision(
                status=DecisionStatus.VETOED,
                command=MotionCommand.create_stop(priority=3, source="health_gate"),
                reason=f"Perception unusable: camera health score {camera_health.score:.2f} < 0.25",
                trigger="CAMERA_UNUSABLE",
                telemetry_snapshot={"camera_health": camera_health.model_dump()},
            )

        if camera_health.is_degraded or camera_health.score < self.min_health:
            # Degraded perception: derate speed to 50%
            scaled_linear = command.linear * 0.5
            scaled_angular = command.angular * 0.7
            scaled_cmd = command.model_copy(
                update={"linear": scaled_linear, "angular": scaled_angular}
            )
            return True, SafetyDecision(
                status=DecisionStatus.SCALED,
                command=scaled_cmd,
                reason=f"Perception degraded (health={camera_health.score:.2f}): linear speed scaled 50%",
                trigger="CAMERA_DEGRADED",
            )

        return True, None
