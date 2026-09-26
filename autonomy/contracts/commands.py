"""Motion command and control arbitration contracts."""

import time
from typing import Optional
from pydantic import BaseModel, Field


class MotionCommand(BaseModel):
    """Normalized motion command with TTL enforcement."""

    linear: float = Field(ge=-1.0, le=1.0, description="Normalized linear velocity in [-1, 1]")
    angular: float = Field(ge=-1.0, le=1.0, description="Normalized angular velocity in [-1, 1]")

    valid_from: float = Field(description="Epoch timestamp when command becomes valid")
    expires_at: float = Field(description="Epoch timestamp when command expires (TTL)")

    source: str = Field(default="local", description="Command origin, e.g. 'edge_mppi', 'remote_mission'")
    priority: int = Field(default=100, description="Lower values indicate higher priority")
    safety_token: Optional[str] = Field(default=None, description="Security/arbitration token if required")

    def is_expired(self, current_time: Optional[float] = None) -> bool:
        """Check if command has expired."""
        now = time.time() if current_time is None else current_time
        return now > self.expires_at

    def is_moving(self) -> bool:
        """Check if command instructs non-zero motion."""
        return abs(self.linear) > 1e-4 or abs(self.angular) > 1e-4

    @classmethod
    def create_stop(cls, priority: int = 0, source: str = "safety_stop", ttl_s: float = 1.0) -> "MotionCommand":
        """Create a zero-motion safety stop command."""
        now = time.time()
        return cls(
            linear=0.0,
            angular=0.0,
            valid_from=now,
            expires_at=now + ttl_s,
            source=source,
            priority=priority,
        )
