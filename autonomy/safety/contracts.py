"""Safety contracts and decision models."""

from enum import Enum
from typing import Any, Dict, Optional
from pydantic import BaseModel, Field
from autonomy.contracts.commands import MotionCommand


class DecisionStatus(str, Enum):
    """Result of command safety arbitration."""

    ALLOWED = "ALLOWED"
    SCALED = "SCALED"
    VETOED = "VETOED"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class RolloverRiskLevel(str, Enum):
    """Vehicle attitude stability classification."""

    NORMAL = "NORMAL"
    CAUTION = "CAUTION"
    HIGH_RISK = "HIGH_RISK"
    EMERGENCY = "EMERGENCY"


class SafetyDecision(BaseModel):
    """Detailed output of command safety evaluation."""

    status: DecisionStatus
    command: MotionCommand
    reason: str = ""
    trigger: Optional[str] = None
    telemetry_snapshot: Dict[str, Any] = Field(default_factory=dict)
