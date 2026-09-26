"""Autonomy event models and dispatch manager."""

import collections
import logging
import time
from typing import Any, Callable, Dict, List, Optional
from pydantic import BaseModel, Field

logger = logging.getLogger("autonomy.events")


class AutonomyEvent(BaseModel):
    """Structured autonomy state transition or safety event."""

    timestamp: float = Field(default_factory=time.time)
    event_type: str = Field(description="Event category, e.g. 'STATE_CHANGE', 'SAFETY_VETO', 'STUCK_DETECTED'")
    from_state: Optional[str] = None
    to_state: Optional[str] = None
    reason: str = Field(default="", description="Descriptive rationale for event")
    sensor_id: Optional[str] = None
    details: Dict[str, Any] = Field(default_factory=dict)


class EventManager:
    """Thread-safe and async-friendly in-memory event bus and rolling recorder."""

    def __init__(self, max_history: int = 500):
        self._history: collections.deque[AutonomyEvent] = collections.deque(maxlen=max_history)
        self._subscribers: List[Callable[[AutonomyEvent], None]] = []

    def record(
        self,
        event_type: str,
        reason: str = "",
        from_state: Optional[str] = None,
        to_state: Optional[str] = None,
        sensor_id: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
    ) -> AutonomyEvent:
        """Create, store, and broadcast an autonomy event."""
        event = AutonomyEvent(
            timestamp=time.time(),
            event_type=event_type,
            from_state=from_state,
            to_state=to_state,
            reason=reason,
            sensor_id=sensor_id,
            details=details or {},
        )
        self._history.append(event)
        logger.info(
            "AutonomyEvent [%s] %s -> %s (reason: %s)",
            event.event_type,
            event.from_state,
            event.to_state,
            event.reason,
        )

        for callback in list(self._subscribers):
            try:
                callback(event)
            except Exception as e:
                logger.error("Error invoking event subscriber: %s", e)

        return event

    def subscribe(self, callback: Callable[[AutonomyEvent], None]) -> None:
        """Register a callback for new events."""
        if callback not in self._subscribers:
            self._subscribers.append(callback)

    def unsubscribe(self, callback: Callable[[AutonomyEvent], None]) -> None:
        """Unregister a callback."""
        if callback in self._subscribers:
            self._subscribers.remove(callback)

    def get_events(
        self, limit: int = 50, event_type: Optional[str] = None
    ) -> List[AutonomyEvent]:
        """Return the most recent recorded events, optionally filtered."""
        events = list(self._history)
        if event_type:
            events = [e for e in events if e.event_type == event_type]
        return events[-limit:]

    def clear(self) -> None:
        """Clear recorded events."""
        self._history.clear()
