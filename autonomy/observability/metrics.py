"""Autonomy operational metrics and telemetry counters."""

import collections
import time
from typing import Dict, Optional


class AutonomyMetrics:
    """In-memory telemetry counters and latency tracking for autonomy."""

    def __init__(self, window_size: int = 100):
        self.started_at: float = time.time()
        self.commands_total: int = 0
        self.commands_vetoed_total: int = 0
        self.deadline_misses_total: int = 0
        self.state_transitions_total: int = 0
        self.safety_stops_total: int = 0

        self._latencies: collections.deque[float] = collections.deque(maxlen=window_size)
        self._last_command_time: Optional[float] = None

    def record_command(self, latency_ms: float = 0.0, vetoed: bool = False) -> None:
        """Record a planned command evaluation."""
        self.commands_total += 1
        self._last_command_time = time.time()
        if vetoed:
            self.commands_vetoed_total += 1
        if latency_ms > 0:
            self._latencies.append(latency_ms)

    def record_deadline_miss(self) -> None:
        """Record when a planner or control cycle misses its deadline."""
        self.deadline_misses_total += 1

    def record_transition(self) -> None:
        """Record an autonomy state transition."""
        self.state_transitions_total += 1

    def record_safety_stop(self) -> None:
        """Record an emergency or safety stop event."""
        self.safety_stops_total += 1

    def snapshot(self) -> Dict[str, object]:
        """Return a dictionary snapshot of operational metrics."""
        avg_latency = (
            sum(self._latencies) / len(self._latencies) if self._latencies else 0.0
        )
        uptime_s = time.time() - self.started_at
        return {
            "uptime_s": round(uptime_s, 2),
            "commands_total": self.commands_total,
            "commands_vetoed_total": self.commands_vetoed_total,
            "deadline_misses_total": self.deadline_misses_total,
            "state_transitions_total": self.state_transitions_total,
            "safety_stops_total": self.safety_stops_total,
            "avg_command_latency_ms": round(avg_latency, 2),
            "last_command_age_s": (
                round(time.time() - self._last_command_time, 2)
                if self._last_command_time
                else None
            ),
        }
