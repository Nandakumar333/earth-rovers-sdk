"""Multi-sensor kinematic state estimator for edge rover autonomy."""

import math
import time
from typing import Dict, List, Optional

from autonomy.contracts.state import (
    PoseEstimate,
    Quaternion,
    RoverState,
    Vector3,
    VelocityEstimate,
)
from autonomy.localization.contracts import (
    SensorSourceHealth,
    TrackingQuality,
    VOEstimate,
)


def _euler_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float) -> Quaternion:
    """Convert roll, pitch, yaw in radians to unit quaternion."""
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return Quaternion(x=x, y=y, z=z, w=w)


class StateEstimator:
    """Kinematic state estimator fusing IMU, wheel odometry, visual odometry, and GPS."""

    def __init__(self, initial_timestamp: Optional[float] = None):
        self._last_timestamp = initial_timestamp or time.monotonic()

        # Pose in local metric frame
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0

        # Orientation in degrees
        self._roll_deg = 0.0
        self._pitch_deg = 0.0
        self._yaw_deg = 0.0

        # Velocities in body frame
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0  # yaw rate (rad/s)

        # Acceleration in body frame
        self._ax = 0.0
        self._ay = 0.0
        self._az = 0.0

        # Diagnostics & sensors
        self._wheel_rpm: List[float] = []
        self._slip_ratio: Optional[float] = None
        self._gps_fix_quality: Optional[float] = None
        self._battery_pct: Optional[float] = None

        # Positional uncertainty covariance [xx, yy, zz, roll, pitch, yaw]
        self._covariance: List[float] = [0.05, 0.05, 0.01, 0.01, 0.01, 0.02]

        # Health tracking
        self.health = SensorSourceHealth()

    def update_imu(
        self,
        timestamp: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        gyro_z: float = 0.0,
        accel_x: float = 0.0,
        accel_y: float = 0.0,
        accel_z: float = 0.0,
    ) -> None:
        """Fuse high-rate IMU attitude, angular rates, and accelerations."""
        dt = max(timestamp - self._last_timestamp, 1e-4)

        self._roll_deg = roll_deg
        self._pitch_deg = pitch_deg
        self._yaw_deg = yaw_deg

        self._wz = gyro_z
        self._ax = accel_x
        self._ay = accel_y
        self._az = accel_z

        self.health.last_imu_update = timestamp
        self.health.imu_health = 1.0
        self._last_timestamp = timestamp

    def update_wheel_odometry(
        self,
        timestamp: float,
        wheel_rpm: List[float],
        linear_speed_mps: float,
        slip_ratio: Optional[float] = None,
    ) -> None:
        """Fuse wheel speeds and encoder-derived velocity."""
        dt = max(timestamp - self._last_timestamp, 1e-4)
        self._wheel_rpm = list(wheel_rpm)
        self._slip_ratio = slip_ratio

        # If wheel slip is high or VO is active, reduce wheel weighting
        weight = 1.0
        if slip_ratio is not None and slip_ratio > 0.4:
            weight = max(1.0 - slip_ratio, 0.1)

        # Update forward velocity if VO is not solely dominating
        if self.health.vo_health < 0.5:
            self._vx = linear_speed_mps * weight

        # Kinematic dead-reckoning integration
        yaw_rad = math.radians(self._yaw_deg)
        dx = self._vx * math.cos(yaw_rad) * dt
        dy = self._vx * math.sin(yaw_rad) * dt

        self._x += dx
        self._y += dy

        # Position covariance grows slowly under dead-reckoning
        self._covariance[0] += 0.005 * dt
        self._covariance[1] += 0.005 * dt

        self.health.last_wheel_update = timestamp
        self.health.wheel_health = 1.0
        self._last_timestamp = timestamp

    def update_vo(self, vo_estimate: VOEstimate) -> None:
        """Fuse visual odometry motion delta and tracking quality."""
        timestamp = vo_estimate.timestamp
        dt = max(timestamp - self._last_timestamp, 1e-4)

        self.health.last_vo_update = timestamp
        self.health.vo_health = vo_estimate.camera_health

        if vo_estimate.tracking_quality == TrackingQuality.LOST:
            # VO is lost; do not integrate motion delta
            self.health.vo_health = 0.0
            return

        if vo_estimate.tracking_quality == TrackingQuality.DEGRADED:
            # Dampen VO weight when degraded
            vo_weight = 0.2
        else:
            vo_weight = 0.8

        # Blend velocities
        self._vx = (1.0 - vo_weight) * self._vx + vo_weight * vo_estimate.linear_velocity.x
        self._vy = (1.0 - vo_weight) * self._vy + vo_weight * vo_estimate.linear_velocity.y

        # Integrate planar motion
        yaw_rad = math.radians(self._yaw_deg)
        dx = (vo_estimate.delta_position.x * math.cos(yaw_rad)
              - vo_estimate.delta_position.y * math.sin(yaw_rad))
        dy = (vo_estimate.delta_position.x * math.sin(yaw_rad)
              + vo_estimate.delta_position.y * math.cos(yaw_rad))

        self._x += dx * vo_weight
        self._y += dy * vo_weight
        self._yaw_deg = (self._yaw_deg + math.degrees(vo_estimate.delta_yaw_rad * vo_weight)) % 360.0

        # Update covariance using VO covariance
        self._covariance[0] = 0.7 * self._covariance[0] + 0.3 * vo_estimate.covariance[0]
        self._covariance[1] = 0.7 * self._covariance[1] + 0.3 * vo_estimate.covariance[1]

        self._last_timestamp = timestamp

    def update_gps(
        self,
        timestamp: float,
        fix_quality: float,
        accuracy_m: float = 5.0,
    ) -> None:
        """Fuse optional GPS health and fix quality."""
        self._gps_fix_quality = fix_quality
        self.health.last_gps_update = timestamp
        self.health.gps_health = min(max(fix_quality, 0.0), 1.0)

    def check_staleness(self, current_time: float) -> None:
        """Decay health metrics for sensors that have stopped transmitting."""
        if current_time - self.health.last_imu_update > 0.5:
            self.health.imu_health = max(0.0, self.health.imu_health - 0.2)
        if current_time - self.health.last_wheel_update > 0.5:
            self.health.wheel_health = max(0.0, self.health.wheel_health - 0.2)
        if current_time - self.health.last_vo_update > 0.5:
            self.health.vo_health = max(0.0, self.health.vo_health - 0.2)

    def get_rover_state(self, current_time: Optional[float] = None) -> RoverState:
        """Construct canonical RoverState snapshot exposing all required fields."""
        now = current_time or self._last_timestamp
        self.check_staleness(now)

        roll_rad = math.radians(self._roll_deg)
        pitch_rad = math.radians(self._pitch_deg)
        yaw_rad = math.radians(self._yaw_deg)
        orientation = _euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

        pose = PoseEstimate(
            position=Vector3(x=self._x, y=self._y, z=self._z),
            orientation=orientation,
            covariance=list(self._covariance),
        )

        velocity = VelocityEstimate(
            linear=Vector3(x=self._vx, y=self._vy, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=self._wz),
            covariance=[0.02] * 6,
        )

        overall_confidence = max(
            0.1,
            (0.4 * self.health.imu_health + 0.3 * self.health.wheel_health + 0.3 * self.health.vo_health)
        )

        return RoverState(
            timestamp=now,
            pose=pose,
            velocity=velocity,
            acceleration=Vector3(x=self._ax, y=self._ay, z=self._az),
            orientation=orientation,
            roll_deg=self._roll_deg,
            pitch_deg=self._pitch_deg,
            yaw_deg=self._yaw_deg,
            wheel_rpm=self._wheel_rpm,
            slip_ratio=self._slip_ratio,
            battery_pct=self._battery_pct,
            gps_fix_quality=self._gps_fix_quality,
            telemetry_age_ms=max(0.0, (now - self.health.last_wheel_update) * 1000.0),
            camera_age_ms=max(0.0, (now - self.health.last_vo_update) * 1000.0),
            state_confidence=overall_confidence,
        )
