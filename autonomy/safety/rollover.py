"""Compound dynamic rollover risk estimator."""

import math
from typing import Tuple
from autonomy.config import SafetyConfig
from autonomy.contracts.state import RoverState
from autonomy.safety.contracts import RolloverRiskLevel


class RolloverEstimator:
    """Estimates rover rollover risk using multi-axis attitudes and dynamic rates."""

    def __init__(self, config: SafetyConfig):
        self.config = config
        self.max_roll = config.max_roll_deg
        self.max_pitch = config.max_pitch_deg
        self.max_slope = config.max_slope_deg

    def evaluate(self, state: RoverState) -> Tuple[RolloverRiskLevel, float, str]:
        """Compute compound angle and classify rollover hazard band."""
        roll = abs(state.roll_deg)
        pitch = abs(state.pitch_deg)

        # Compound vehicle attitude angle
        compound_deg = math.sqrt(roll**2 + pitch**2)

        # Dynamic angular rate compensation
        # Higher angular velocity (turn or tip rate) multiplies risk
        angular_rate = math.sqrt(
            state.velocity.angular.x**2
            + state.velocity.angular.y**2
            + state.velocity.angular.z**2
        )
        dynamic_factor = 1.0 + min(angular_rate * 0.15, 0.5)
        effective_angle = compound_deg * dynamic_factor

        # Threshold bands based on vehicle capability (18° operational envelope)
        if effective_angle >= max(self.max_roll, self.max_pitch) or roll >= self.max_roll or pitch >= self.max_pitch:
            level = RolloverRiskLevel.EMERGENCY
            reason = f"Extreme attitude: roll={roll:.1f}°, pitch={pitch:.1f}°, compound={compound_deg:.1f}°"
        elif effective_angle >= self.max_slope or compound_deg >= 16.0:
            level = RolloverRiskLevel.HIGH_RISK
            reason = f"High rollover risk: compound angle {compound_deg:.1f}° near maximum slope {self.max_slope:.1f}°"
        elif effective_angle >= 12.0 or compound_deg >= 12.0:
            level = RolloverRiskLevel.CAUTION
            reason = f"Caution attitude: compound angle {compound_deg:.1f}°"
        else:
            level = RolloverRiskLevel.NORMAL
            reason = f"Stable attitude: compound angle {compound_deg:.1f}°"

        return level, compound_deg, reason
