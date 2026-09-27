"""Contracts and data models for localization and visual odometry."""

from enum import Enum
from typing import Dict, List, Optional
from pydantic import BaseModel, Field

from autonomy.contracts.state import PoseEstimate, Vector3


class TrackingQuality(str, Enum):
    """Quality status of visual tracking."""

    OPTIMAL = "OPTIMAL"
    DEGRADED = "DEGRADED"
    LOST = "LOST"


class VOEstimate(BaseModel):
    """Motion estimate produced by Visual Odometry between camera frames."""

    timestamp: float
    delta_position: Vector3 = Field(default_factory=Vector3)
    delta_yaw_rad: float = 0.0
    linear_velocity: Vector3 = Field(default_factory=Vector3)
    angular_velocity: Vector3 = Field(default_factory=Vector3)
    feature_count: int = 0
    inlier_ratio: float = 1.0
    tracking_quality: TrackingQuality = TrackingQuality.OPTIMAL
    covariance: List[float] = Field(default_factory=lambda: [0.01] * 6)
    camera_health: float = 1.0


class SensorSourceHealth(BaseModel):
    """Health metrics and telemetry freshness across localization sensors."""

    imu_health: float = 1.0
    wheel_health: float = 1.0
    vo_health: float = 0.0
    gps_health: float = 0.0

    last_imu_update: float = 0.0
    last_wheel_update: float = 0.0
    last_vo_update: float = 0.0
    last_gps_update: float = 0.0

    def as_dict(self) -> Dict[str, float]:
        return {
            "imu": self.imu_health,
            "wheel": self.wheel_health,
            "vo": self.vo_health,
            "gps": self.gps_health,
        }
