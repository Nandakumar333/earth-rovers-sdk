"""Unit tests for autonomy events and metrics observability."""

from autonomy.observability.events import EventManager
from autonomy.observability.metrics import AutonomyMetrics


def test_event_manager_recording_and_retrieval():
    em = EventManager(max_history=5)
    for i in range(7):
        em.record(event_type="TEST_EVENT", reason=f"iteration_{i}")

    # Retains at most max_history events
    events = em.get_events()
    assert len(events) == 5
    assert events[-1].reason == "iteration_6"
    assert events[0].reason == "iteration_2"


def test_event_manager_filtering():
    em = EventManager()
    em.record(event_type="STATE_TRANSITION", reason="boot")
    em.record(event_type="SAFETY_VETO", reason="steep_slope")
    em.record(event_type="STATE_TRANSITION", reason="ready")

    state_events = em.get_events(event_type="STATE_TRANSITION")
    assert len(state_events) == 2
    assert state_events[0].reason == "boot"
    assert state_events[1].reason == "ready"

    veto_events = em.get_events(event_type="SAFETY_VETO")
    assert len(veto_events) == 1
    assert veto_events[0].reason == "steep_slope"


def test_event_manager_subscribers():
    em = EventManager()
    captured = []

    def on_event(event):
        captured.append(event)

    em.subscribe(on_event)
    em.record(event_type="ALERT", reason="low_battery")

    assert len(captured) == 1
    assert captured[0].reason == "low_battery"

    em.unsubscribe(on_event)
    em.record(event_type="ALERT", reason="lost_gps")
    assert len(captured) == 1


def test_autonomy_metrics():
    metrics = AutonomyMetrics()
    metrics.record_command(latency_ms=25.0, vetoed=False)
    metrics.record_command(latency_ms=35.0, vetoed=True)
    metrics.record_deadline_miss()
    metrics.record_transition()
    metrics.record_safety_stop()

    snap = metrics.snapshot()
    assert snap["commands_total"] == 2
    assert snap["commands_vetoed_total"] == 1
    assert snap["deadline_misses_total"] == 1
    assert snap["state_transitions_total"] == 1
    assert snap["safety_stops_total"] == 1
    assert snap["avg_command_latency_ms"] == 30.0
    assert snap["last_command_age_s"] is not None
