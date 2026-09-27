"""Wheel slip estimation and vehicle stuck condition detection."""

import math
import time
from typing import Optional, Tuple
from autonomy.config import SafetyConfig
from autonomy.contracts.commands import MotionCommand
from autonomy.contracts.state import RoverState


class StuckDetector:
    """Detects lack of forward vehicle progress despite active motor commands."""

    def __init__(self, config: SafetyConfig):
        self.config = config
        self.window_s = config.stuck_detection_window_s
        self._stuck_start_time: Optional[float] = None
        self._command_threshold = 0.15
        self._motion_threshold = 0.05

    def compute_slip(self, state: RoverState) -> float:
        """Calculate normalized wheel slip ratio in [0, 1] with zero-division guard."""
        # Mean wheel linear velocity approximation from RPM
        if not state.wheel_rpm:
            return 0.0

        avg_rpm = sum(abs(r) for r in state.wheel_rpm) / len(state.wheel_rpm)
        # Approximate wheel surface speed (m/s) assuming MINI wheel radius ~ 0.06m
        v_wheel = (avg_rpm * 2.0 * math.pi / 60.0) * 0.06

        if v_wheel < 1e-3:
            return 0.0

        # Actual velocity magnitude
        v_actual = math.sqrt(
            state.velocity.linear.x**2
            + state.velocity.linear.y**2
            + state.velocity.linear.z**2
        )

        slip = 1.0 - (v_actual / v_wheel)
        return max(0.0, min(slip, 1.0))

    def evaluate(
        self, command: MotionCommand, state: RoverState, current_time: Optional[float] = None
    ) -> Tuple[bool, float, str]:
        """Check whether rover is experiencing a stuck condition."""
        now = time.time() if current_time is None else current_time
        slip = self.compute_slip(state)

        command_effort = abs(command.linear)
        actual_speed = math.sqrt(
            state.velocity.linear.x**2
            + state.velocity.linear.y**2
            + state.velocity.linear.z**2
        )

        # Rover is commanding meaningful drive, but actual motion is near zero
        is_failing_progress = (
            command_effort > self._command_threshold
            and actual_speed < self._motion_threshold
        )

        if is_failing_progress:
            if self._stuck_start_time is None:
                self._stuck_start_time = now

            duration = now - self._stuck_start_time
            if duration >= self.window_s:
                reason = (
                    f"Rover stuck: commanded linear={command.linear:.2f} for {duration:.1f}s "
                    f"with actual speed={actual_speed:.3f}m/s (slip={slip:.2f})"
                )
                return True, slip, reason
        else:
            # Progress restored or stopped commanding drive
            self._stuck_start_time = None

        return False, slip, "Normal traction"

    def reset(self) -> None:
        """Reset internal timer."""
        self._stuck_start_time = None
