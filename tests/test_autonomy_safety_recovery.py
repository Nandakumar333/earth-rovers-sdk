"""Unit tests for recovery state machine."""

import pytest
from autonomy.safety.recovery import RecoveryState, RecoveryStateMachine


def test_recovery_lifecycle():
    rsm = RecoveryStateMachine(max_attempts=2)
    assert rsm.state == RecoveryState.IDLE
    assert not rsm.is_active

    # Stuck event triggered
    rsm.trigger_stuck(reason="high slip in gravel")
    assert rsm.state == RecoveryState.STOP
    assert rsm.is_active

    # Step 1: Delivers settling stop
    cmd1 = rsm.step(current_time=100.0)
    assert cmd1 is not None
    assert cmd1.linear == 0.0
    assert rsm.state == RecoveryState.VERIFY

    # Step 2: Delivers maneuver pulse
    cmd2 = rsm.step(current_time=100.5)
    assert cmd2 is not None
    assert cmd2.linear < 0  # reverse pulse
    assert cmd2.expires_at - cmd2.valid_from <= 0.5
    assert rsm.state == RecoveryState.EXECUTING_MANEUVER

    # Step 3: Fast-forward maneuver duration (400ms)
    cmd3 = rsm.step(current_time=101.0)
    assert rsm.state == RecoveryState.VERIFY_PROGRESS

    # Traction restored: resolve
    rsm.resolve()
    assert rsm.state == RecoveryState.IDLE
    assert not rsm.is_active


def test_recovery_escalation():
    rsm = RecoveryStateMachine(max_attempts=1)
    rsm.trigger_stuck(reason="stuck")
    rsm.step(current_time=100.0)  # STOP -> VERIFY
    rsm.step(current_time=100.1)  # VERIFY -> EXECUTING_MANEUVER
    rsm.step(current_time=100.6)  # EXECUTING -> VERIFY_PROGRESS

    # Still stuck: triggers second attempt, which exceeds max_attempts (1)
    rsm.trigger_stuck(reason="still stuck")
    assert rsm.state == RecoveryState.ESCALATED
    cmd = rsm.step(current_time=101.0)
    assert cmd is not None
    assert cmd.linear == 0.0  # Safe zero stop
