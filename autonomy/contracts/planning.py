"""Terrain modeling and trajectory planning contracts."""

from typing import List, Optional
from pydantic import BaseModel, Field
from autonomy.contracts.commands import MotionCommand


class TerrainCell(BaseModel):
    """Local grid terrain cell with cost components and traversability."""

    x: float
    y: float

    elevation_m: Optional[float] = None
    slope_deg: Optional[float] = None
    roughness: Optional[float] = None
    curvature: Optional[float] = None

    obstacle_cost: float = Field(default=0.0, ge=0.0, le=1.0)
    dropoff_cost: float = Field(default=0.0, ge=0.0, le=1.0)
    slip_cost: float = Field(default=0.0, ge=0.0, le=1.0)
    rollover_cost: float = Field(default=0.0, ge=0.0, le=1.0)
    uncertainty_cost: float = Field(default=0.0, ge=0.0, le=1.0)

    traversability: float = Field(default=0.0, ge=0.0, le=1.0)
    confidence: float = Field(default=1.0, ge=0.0, le=1.0)


class TrajectoryPoint(BaseModel):
    """Single step along an optimized trajectory."""

    timestamp: float
    x: float
    y: float
    yaw_rad: float
    linear_velocity: float
    angular_velocity: float
    cost: float = 0.0


class TrajectoryPlan(BaseModel):
    """Sequence of planned trajectory points and resulting immediate command."""

    generated_at: float
    horizon_s: float
    points: List[TrajectoryPoint] = Field(default_factory=list)
    immediate_command: Optional[MotionCommand] = None
    cost: float = 0.0
