"""Autonomy configuration models and loader."""

import os
from pathlib import Path
from typing import Optional
import yaml
from pydantic import BaseModel, Field


class SafetyConfig(BaseModel):
    """Safety and command arbitration thresholds."""

    command_ttl_ms: float = Field(default=500.0, description="Max command freshness in ms")
    control_rate_hz: float = Field(default=10.0, description="Target control dispatch rate in Hz")
    max_slope_deg: float = Field(default=18.0, description="Maximum vehicle operational slope in degrees")
    max_roll_deg: float = Field(default=20.0, description="Critical roll angle threshold in degrees")
    max_pitch_deg: float = Field(default=20.0, description="Critical pitch angle threshold in degrees")
    stuck_detection_window_s: float = Field(default=2.0, description="Duration before declaring stuck in seconds")
    min_obstacle_clearance_m: float = Field(default=0.35, description="Min distance to obstacles in meters")


class NetworkConfig(BaseModel):
    """Network health and latency tolerances."""

    disconnect_timeout_s: float = Field(default=3.0, description="Seconds without telemetry before safe stop")
    max_rtt_ms: float = Field(default=1000.0, description="RTT upper bound before degradation")
    max_jitter_ms: float = Field(default=300.0, description="Jitter upper bound before degradation")


class PlanningConfig(BaseModel):
    """Local trajectory planner configuration."""

    horizon_s: float = Field(default=1.5, description="Local trajectory planning horizon in seconds")
    num_samples: int = Field(default=128, description="Number of candidate trajectories to evaluate")
    max_linear_speed: float = Field(default=0.8, description="Normalized max linear speed")
    max_angular_speed: float = Field(default=0.7, description="Normalized max angular speed")


class SensorsConfig(BaseModel):
    """Sensor ingestion and health parameters."""

    camera_fps: int = Field(default=30, description="Camera feed capture rate")
    min_camera_health: float = Field(default=0.5, description="Minimum acceptable camera health score")
    imu_source: str = Field(default="sdk", description="IMU source provider ('sdk', 'ros2', 'serial')")


class AutonomyConfig(BaseModel):
    """Root configuration for autonomy layer."""

    enabled: bool = Field(default=False, description="Master feature flag for autonomy")
    mode: str = Field(default="hybrid", description="Deployment mode: 'edge', 'remote', or 'hybrid'")
    safety: SafetyConfig = Field(default_factory=SafetyConfig)
    network: NetworkConfig = Field(default_factory=NetworkConfig)
    planning: PlanningConfig = Field(default_factory=PlanningConfig)
    sensors: SensorsConfig = Field(default_factory=SensorsConfig)


def load_autonomy_config(config_path: Optional[str] = None) -> AutonomyConfig:
    """Load autonomy configuration from YAML with environment variable overrides."""
    data = {}

    default_path = Path("configs/autonomy.yaml")
    target_path = Path(config_path) if config_path else default_path

    if target_path.exists():
        try:
            with open(target_path, "r", encoding="utf-8") as f:
                loaded = yaml.safe_load(f)
                if isinstance(loaded, dict):
                    data = loaded
        except Exception:
            # Fallback to empty dict if loading fails
            data = {}

    # Environment variable overrides
    env_enabled = os.getenv("AUTONOMY_ENABLED")
    if env_enabled is not None:
        data["enabled"] = env_enabled.lower() in ("true", "1", "yes")

    env_mode = os.getenv("AUTONOMY_MODE")
    if env_mode:
        data["mode"] = env_mode

    # Safety overrides
    safety_data = data.get("safety", {})
    if os.getenv("AUTONOMY_COMMAND_TTL_MS"):
        safety_data["command_ttl_ms"] = float(os.environ["AUTONOMY_COMMAND_TTL_MS"])
    if os.getenv("AUTONOMY_CONTROL_RATE_HZ"):
        safety_data["control_rate_hz"] = float(os.environ["AUTONOMY_CONTROL_RATE_HZ"])
    if os.getenv("AUTONOMY_MAX_SLOPE_DEG"):
        safety_data["max_slope_deg"] = float(os.environ["AUTONOMY_MAX_SLOPE_DEG"])
    if safety_data:
        data["safety"] = safety_data

    return AutonomyConfig(**data)
