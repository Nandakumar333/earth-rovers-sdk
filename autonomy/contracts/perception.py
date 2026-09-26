"""Perception observation and visual health contracts."""

from typing import Any, Dict
from pydantic import BaseModel, Field


class PerceptionObservation(BaseModel):
    """Generic perception sensor observation with uncertainty metadata."""

    timestamp: float
    sensor_id: str
    frame_id: str

    payload: Any = None

    confidence: float = Field(default=1.0, ge=0.0, le=1.0)
    age_ms: float = Field(default=0.0, ge=0.0)
    quality: float = Field(default=1.0, ge=0.0, le=1.0)

    uncertainty: Dict[str, float] = Field(default_factory=dict)


class CameraHealthScore(BaseModel):
    """Assessment of camera stream quality and degradation."""

    score: float = Field(ge=0.0, le=1.0, description="Overall health score in [0, 1]")
    blur_score: float = Field(ge=0.0, default=1.0)
    brightness: float = Field(ge=0.0, le=255.0, default=128.0)
    contrast: float = Field(ge=0.0, default=1.0)
    is_degraded: bool = False
    is_usable: bool = True
