"""Unit and integration tests for SafetyArbiter and Runtime arbitration."""

import time
import pytest
from autonomy.config import AutonomyConfig
from autonomy.contracts.commands import MotionCommand
from autonomy.contracts.perception import CameraHealthScore
from autonomy.contracts.state import AutonomyState, RoverState, VelocityEstimate, Vector3
from autonomy.runtime import AutonomyRuntime
from autonomy.safety.arbiter import SafetyArbiter
from autonomy.safety.contracts import DecisionStatus


@pytest.fixture
def autonomy_setup():
    config = AutonomyConfig(enabled=True)
    runtime = AutonomyRuntime(config=config)
    return runtime, runtime.arbiter


def test_arbiter_nominal_allow(autonomy_setup):
    runtime, arbiter = autonomy_setup
    now = time.time()
    cmd = MotionCommand(linear=0.6, angular=0.1, valid_from=now, expires_at=now + 0.4)
    state = RoverState(timestamp=now, telemetry_age_ms=50.0, roll_deg=2.0, pitch_deg=3.0)

    decision = arbiter.arbitrate(cmd, state, current_time=now)
    assert decision.status == DecisionStatus.ALLOWED
    assert decision.command.linear == 0.6
    assert decision.command.angular == 0.1


def test_arbiter_ttl_watchdog_veto(autonomy_setup):
    runtime, arbiter = autonomy_setup
    now = time.time()
    # Expired command
    cmd = MotionCommand(linear=0.5, angular=0.0, valid_from=now - 2.0, expires_at=now - 0.5)
    state = RoverState(timestamp=now, telemetry_age_ms=50.0)

    decision = arbiter.arbitrate(cmd, state, current_time=now)
    assert decision.status == DecisionStatus.VETOED
    assert decision.trigger == "TTL_EXPIRED"
    assert decision.command.linear == 0.0


def test_arbiter_network_watchdog_veto(autonomy_setup):
    runtime, arbiter = autonomy_setup
    now = time.time()
    cmd = MotionCommand(linear=0.5, angular=0.0, valid_from=now, expires_at=now + 0.4)
    # Telemetry is 4 seconds old (exceeds default 3.0s disconnect timeout)
    state = RoverState(timestamp=now, telemetry_age_ms=4000.0)

    decision = arbiter.arbitrate(cmd, state, current_time=now)
    assert decision.status == DecisionStatus.VETOED
    assert decision.trigger == "NETWORK_DISCONNECTED"
    assert decision.command.linear == 0.0


def test_arbiter_rollover_emergency(autonomy_setup):
    runtime, arbiter = autonomy_setup
    now = time.time()
    cmd = MotionCommand(linear=0.7, angular=0.0, valid_from=now, expires_at=now + 0.4)
    # Critical roll angle 22° > 20°
    state = RoverState(timestamp=now, telemetry_age_ms=50.0, roll_deg=22.0, pitch_deg=4.0)

    decision = arbiter.arbitrate(cmd, state, current_time=now)
    assert decision.status == DecisionStatus.EMERGENCY_STOP
    assert decision.trigger == "ROLLOVER_EMERGENCY"
    assert decision.command.linear == 0.0


def test_arbiter_rollover_caution_derating(autonomy_setup):
    runtime, arbiter = autonomy_setup
    now = time.time()
    cmd = MotionCommand(linear=0.8, angular=0.2, valid_from=now, expires_at=now + 0.4)
    # Compound angle ~ 14.1° -> CAUTION band
    state = RoverState(timestamp=now, telemetry_age_ms=50.0, roll_deg=10.0, pitch_deg=10.0)

    decision = arbiter.arbitrate(cmd, state, current_time=now)
    assert decision.status == DecisionStatus.SCALED
    # Linear speed derated to 60%
    assert decision.command.linear == pytest.approx(0.48, 0.01)


def test_arbiter_camera_health_gating(autonomy_setup):
    runtime, arbiter = autonomy_setup
    now = time.time()
    cmd = MotionCommand(linear=0.6, angular=0.2, valid_from=now, expires_at=now + 0.4)
    state = RoverState(timestamp=now, telemetry_age_ms=50.0)

    # 1. Degraded camera health (0.45 < 0.5)
    degraded = CameraHealthScore(score=0.45, is_degraded=True, is_usable=True)
    decision = arbiter.arbitrate(cmd, state, camera_health=degraded, current_time=now)
    assert decision.status == DecisionStatus.SCALED
    assert decision.command.linear == pytest.approx(0.3, 0.01)

    # 2. Unusable camera (score < 0.25)
    unusable = CameraHealthScore(score=0.15, is_degraded=True, is_usable=False)
    decision = arbiter.arbitrate(cmd, state, camera_health=unusable, current_time=now)
    assert decision.status == DecisionStatus.VETOED
    assert decision.trigger == "CAMERA_UNUSABLE"
    assert decision.command.linear == 0.0


@pytest.mark.asyncio
async def test_runtime_arbitrate_integration():
    runtime = AutonomyRuntime(config=AutonomyConfig(enabled=True))
    await runtime.start()
    runtime.transition_to(AutonomyState.AUTONOMOUS, reason="start drive")

    now = time.time()
    # Trigger emergency rollover
    danger_state = RoverState(timestamp=now, roll_deg=25.0)
    cmd = MotionCommand(linear=0.5, angular=0.0, valid_from=now, expires_at=now + 0.4)

    decision = runtime.arbitrate_command(cmd, danger_state, current_time=now)
    assert decision.status == DecisionStatus.EMERGENCY_STOP
    # Runtime transitioned automatically to EMERGENCY_STOP
    assert runtime.state == AutonomyState.EMERGENCY_STOP

    # Events and metrics recorded
    veto_events = runtime.events.get_events(event_type="SAFETY_VETO")
    assert len(veto_events) >= 1
    assert veto_events[-1].details["trigger"] == "ROLLOVER_EMERGENCY"
    assert runtime.metrics.commands_vetoed_total >= 1
    assert runtime.metrics.safety_stops_total >= 1
