"""Unit tests for slip calculation and stuck detection."""

import pytest
from autonomy.config import SafetyConfig
from autonomy.contracts.commands import MotionCommand
from autonomy.contracts.state import RoverState, VelocityEstimate, Vector3
from autonomy.safety.stuck import StuckDetector


@pytest.fixture
def stuck_detector():
    config = SafetyConfig(stuck_detection_window_s=2.0)
    return StuckDetector(config=config)


def test_slip_zero_when_stationary(stuck_detector):
    state = RoverState(
        timestamp=100.0,
        wheel_rpm=[0.0, 0.0, 0.0, 0.0],
        velocity=VelocityEstimate(linear=Vector3(x=0.0, y=0.0, z=0.0)),
    )
    slip = stuck_detector.compute_slip(state)
    assert slip == 0.0


def test_slip_high_wheel_spin(stuck_detector):
    # High RPM, zero ground velocity -> 100% slip
    state = RoverState(
        timestamp=100.0,
        wheel_rpm=[200.0, 200.0, 200.0, 200.0],
        velocity=VelocityEstimate(linear=Vector3(x=0.0, y=0.0, z=0.0)),
    )
    slip = stuck_detector.compute_slip(state)
    assert slip == 1.0


def test_stuck_detection_timing(stuck_detector):
    cmd = MotionCommand(
        linear=0.8,
        angular=0.0,
        valid_from=100.0,
        expires_at=101.0,
    )
    stuck_state = RoverState(
        timestamp=100.0,
        wheel_rpm=[150.0, 150.0, 150.0, 150.0],
        velocity=VelocityEstimate(linear=Vector3(x=0.01, y=0.0, z=0.0)),
    )

    # Initial tick at t=100.0
    is_stuck, slip, _ = stuck_detector.evaluate(cmd, stuck_state, current_time=100.0)
    assert not is_stuck

    # After 1.0s (less than 2.0s window)
    is_stuck, slip, _ = stuck_detector.evaluate(cmd, stuck_state, current_time=101.0)
    assert not is_stuck

    # After 2.1s (exceeds 2.0s window)
    is_stuck, slip, reason = stuck_detector.evaluate(cmd, stuck_state, current_time=102.1)
    assert is_stuck
    assert "Rover stuck" in reason
    assert slip > 0.8


def test_stuck_detection_clears_on_progress(stuck_detector):
    cmd = MotionCommand(linear=0.5, angular=0.0, valid_from=100.0, expires_at=101.0)
    stuck_state = RoverState(
        timestamp=100.0,
        wheel_rpm=[120.0, 120.0],
        velocity=VelocityEstimate(linear=Vector3(x=0.0, y=0.0, z=0.0)),
    )

    stuck_detector.evaluate(cmd, stuck_state, current_time=100.0)
    stuck_detector.evaluate(cmd, stuck_state, current_time=102.5)  # stuck!

    # Rover begins moving at 0.4 m/s
    moving_state = RoverState(
        timestamp=103.0,
        wheel_rpm=[120.0, 120.0],
        velocity=VelocityEstimate(linear=Vector3(x=0.4, y=0.0, z=0.0)),
    )
    is_stuck, _, _ = stuck_detector.evaluate(cmd, moving_state, current_time=103.0)
    assert not is_stuck
