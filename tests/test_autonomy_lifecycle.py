"""Unit tests for autonomy runtime lifecycle and state machine."""

import pytest
from autonomy.config import AutonomyConfig
from autonomy.contracts.state import AutonomyState
from autonomy.runtime import AutonomyRuntime, StateTransitionError


@pytest.mark.asyncio
async def test_runtime_initial_state():
    config = AutonomyConfig(enabled=False)
    runtime = AutonomyRuntime(config=config)
    assert runtime.state == AutonomyState.DISABLED
    assert not runtime.is_autonomous
    assert not runtime.is_active


@pytest.mark.asyncio
async def test_runtime_start_and_stop():
    config = AutonomyConfig(enabled=True)
    runtime = AutonomyRuntime(config=config)

    await runtime.start()
    assert runtime.state == AutonomyState.READY
    assert runtime.is_active

    await runtime.stop()
    assert runtime.state == AutonomyState.DISABLED
    assert not runtime.is_active


@pytest.mark.asyncio
async def test_valid_state_transitions():
    runtime = AutonomyRuntime(config=AutonomyConfig(enabled=True))
    await runtime.start()
    assert runtime.state == AutonomyState.READY

    # READY -> AUTONOMOUS
    runtime.transition_to(AutonomyState.AUTONOMOUS, reason="start mission")
    assert runtime.state == AutonomyState.AUTONOMOUS
    assert runtime.is_autonomous

    # AUTONOMOUS -> DEFERRED
    runtime.transition_to(AutonomyState.DEFERRED, reason="poor visibility")
    assert runtime.state == AutonomyState.DEFERRED

    # DEFERRED -> AUTONOMOUS
    runtime.transition_to(AutonomyState.AUTONOMOUS, reason="visibility restored")
    assert runtime.state == AutonomyState.AUTONOMOUS

    # AUTONOMOUS -> STOP
    runtime.transition_to(AutonomyState.STOP, reason="obstacle detected")
    assert runtime.state == AutonomyState.STOP

    # STOP -> RECOVERY
    runtime.transition_to(AutonomyState.RECOVERY, reason="executing backup maneuver")
    assert runtime.state == AutonomyState.RECOVERY

    # RECOVERY -> AUTONOMOUS
    runtime.transition_to(AutonomyState.AUTONOMOUS, reason="recovery complete")
    assert runtime.state == AutonomyState.AUTONOMOUS


@pytest.mark.asyncio
async def test_illegal_state_transitions():
    runtime = AutonomyRuntime(config=AutonomyConfig(enabled=False))
    assert runtime.state == AutonomyState.DISABLED

    # DISABLED cannot jump directly to AUTONOMOUS
    with pytest.raises(StateTransitionError):
        runtime.transition_to(AutonomyState.AUTONOMOUS, reason="invalid leap")

    # State must be preserved
    assert runtime.state == AutonomyState.DISABLED


@pytest.mark.asyncio
async def test_emergency_stop_and_reset():
    runtime = AutonomyRuntime(config=AutonomyConfig(enabled=True))
    await runtime.start()
    runtime.transition_to(AutonomyState.AUTONOMOUS, reason="running")

    # Emergency stop from AUTONOMOUS
    runtime.emergency_stop(reason="critical rollover imminent")
    assert runtime.state == AutonomyState.EMERGENCY_STOP

    # Cannot transition directly to AUTONOMOUS from EMERGENCY_STOP
    with pytest.raises(StateTransitionError):
        runtime.transition_to(AutonomyState.AUTONOMOUS)

    # Must reset to MANUAL_ONLY
    reset_ok = runtime.reset_emergency(reason="human operator verified rover orientation")
    assert reset_ok is True
    assert runtime.state == AutonomyState.MANUAL_ONLY

    # From MANUAL_ONLY back to READY
    runtime.transition_to(AutonomyState.READY, reason="re-engage autonomy")
    assert runtime.state == AutonomyState.READY


@pytest.mark.asyncio
async def test_runtime_status_snapshot():
    runtime = AutonomyRuntime(config=AutonomyConfig(enabled=True, mode="edge"))
    await runtime.start()

    status = runtime.status()
    assert status["enabled"] is True
    assert status["state"] == "READY"
    assert status["mode"] == "edge"
    assert "metrics" in status
    assert "recent_events" in status
    assert len(status["recent_events"]) >= 1

    await runtime.stop()
