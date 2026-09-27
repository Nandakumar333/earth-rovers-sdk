"""Unit tests for rollover risk estimator."""

import pytest
from autonomy.config import SafetyConfig
from autonomy.contracts.state import RoverState, VelocityEstimate, Vector3
from autonomy.safety.contracts import RolloverRiskLevel
from autonomy.safety.rollover import RolloverEstimator


@pytest.fixture
def rollover_estimator():
    config = SafetyConfig(max_roll_deg=20.0, max_pitch_deg=20.0, max_slope_deg=18.0)
    return RolloverEstimator(config=config)


def test_rollover_normal_attitude(rollover_estimator):
    state = RoverState(
        timestamp=100.0,
        roll_deg=3.0,
        pitch_deg=4.0,
    )
    level, angle, reason = rollover_estimator.evaluate(state)
    assert level == RolloverRiskLevel.NORMAL
    assert angle == pytest.approx(5.0, 0.1)


def test_rollover_caution_attitude(rollover_estimator):
    state = RoverState(
        timestamp=100.0,
        roll_deg=10.0,
        pitch_deg=9.0,
    )
    level, angle, reason = rollover_estimator.evaluate(state)
    assert level == RolloverRiskLevel.CAUTION
    assert 12.0 <= angle < 16.0


def test_rollover_high_risk_attitude(rollover_estimator):
    state = RoverState(
        timestamp=100.0,
        roll_deg=12.0,
        pitch_deg=13.0,
    )
    level, angle, reason = rollover_estimator.evaluate(state)
    assert level == RolloverRiskLevel.HIGH_RISK
    assert angle >= 16.0


def test_rollover_emergency_attitude(rollover_estimator):
    # Exceeding single-axis max (20°)
    state = RoverState(
        timestamp=100.0,
        roll_deg=21.0,
        pitch_deg=2.0,
    )
    level, angle, reason = rollover_estimator.evaluate(state)
    assert level == RolloverRiskLevel.EMERGENCY
    assert "Extreme attitude" in reason


def test_rollover_dynamic_angular_rate_amplification(rollover_estimator):
    # Static compound angle is 11° (normally NORMAL), but rapid angular roll rate escalates it
    state = RoverState(
        timestamp=100.0,
        roll_deg=8.0,
        pitch_deg=7.5,
        velocity=VelocityEstimate(angular=Vector3(x=2.5, y=0.0, z=0.0)),
    )
    level, angle, reason = rollover_estimator.evaluate(state)
    # Dynamic factor escalates effective angle into CAUTION
    assert level in (RolloverRiskLevel.CAUTION, RolloverRiskLevel.HIGH_RISK)
