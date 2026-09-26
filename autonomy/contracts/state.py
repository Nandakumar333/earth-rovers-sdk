"""Kinematic, pose, and rover state contracts."""

from enum import Enum
from typing import List, Optional
from pydantic import BaseModel, Field


class AutonomyState(str, Enum):
    """Runtime autonomy state enumeration."""

    DISABLED = "DISABLED"
    INITIALIZING = "INITIALIZING"
    READY = "READY"
    AUTONOMOUS = "AUTONOMOUS"
    DEFERRED = "DEFERRED"
    STOP = "STOP"
    RECOVERY = "RECOVERY"
    MANUAL_ONLY = "MANUAL_ONLY"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class Vector3(BaseModel):
    """3D Cartesian vector."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class Quaternion(BaseModel):
    """Orientation quaternion."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


class PoseEstimate(BaseModel):
    """Estimated 3D position and orientation with covariance."""

    position: Vector3 = Field(default_factory=Vector3)
    orientation: Quaternion = Field(default_factory=Quaternion)
    covariance: Optional[List[float]] = None


class VelocityEstimate(BaseModel):
    """Estimated linear and angular velocities."""

    linear: Vector3 = Field(default_factory=Vector3)
    angular: Vector3 = Field(default_factory=Vector3)
    covariance: Optional[List[float]] = None


class RoverState(BaseModel):
    """Comprehensive estimated state of the rover."""

    timestamp: float
    pose: PoseEstimate = Field(default_factory=PoseEstimate)
    velocity: VelocityEstimate = Field(default_factory=VelocityEstimate)
    acceleration: Vector3 = Field(default_factory=Vector3)
    orientation: Quaternion = Field(default_factory=Quaternion)
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0

    wheel_rpm: List[float] = Field(default_factory=list)
    slip_ratio: Optional[float] = None

    battery_pct: Optional[float] = None
    gps_fix_quality: Optional[float] = None

    command_age_ms: float = 0.0
    telemetry_age_ms: float = 0.0
    camera_age_ms: float = 0.0

    network_rtt_ms: Optional[float] = None
    network_jitter_ms: Optional[float] = None

    state_confidence: float = 1.0
