"""Tests for multi-sensor StateEstimator."""

import math
import time
import pytest

from autonomy.contracts.state import Vector3
from autonomy.localization.contracts import TrackingQuality, VOEstimate
from autonomy.localization.state_estimator import StateEstimator


def test_state_estimator_initial_state():
    estimator = StateEstimator(initial_timestamp=100.0)
    state = estimator.get_rover_state(current_time=100.0)

    assert state.pose.position.x == 0.0
    assert state.pose.position.y == 0.0
    assert state.orientation.w == 1.0
    assert state.state_confidence >= 0.5


def test_state_estimator_imu_update():
    estimator = StateEstimator(initial_timestamp=100.0)
    estimator.update_imu(
        timestamp=100.1,
        roll_deg=5.0,
        pitch_deg=-3.0,
        yaw_deg=45.0,
        gyro_z=0.2,
        accel_x=0.5,
    )
    state = estimator.get_rover_state(current_time=100.1)

    assert state.roll_deg == 5.0
    assert state.pitch_deg == -3.0
    assert state.yaw_deg == 45.0
    assert state.velocity.angular.z == 0.2
    assert state.acceleration.x == 0.5
    assert estimator.health.imu_health == 1.0


def test_state_estimator_wheel_dead_reckoning():
    estimator = StateEstimator(initial_timestamp=100.0)
    # Heading is 0 deg (facing +X)
    estimator.update_wheel_odometry(
        timestamp=101.0,
        wheel_rpm=[100.0, 100.0],
        linear_speed_mps=1.0,  # 1 m/s for 1 sec
    )
    state = estimator.get_rover_state(current_time=101.0)

    assert pytest.approx(state.pose.position.x, abs=0.05) == 1.0
    assert pytest.approx(state.pose.position.y, abs=0.01) == 0.0
    assert state.velocity.linear.x > 0.0


def test_state_estimator_vo_fusion():
    estimator = StateEstimator(initial_timestamp=100.0)
    vo_est = VOEstimate(
        timestamp=100.2,
        delta_position=Vector3(x=0.2, y=0.0, z=0.0),
        delta_yaw_rad=0.0,
        linear_velocity=Vector3(x=1.0, y=0.0, z=0.0),
        feature_count=30,
        inlier_ratio=0.85,
        tracking_quality=TrackingQuality.OPTIMAL,
        camera_health=0.9,
    )
    estimator.update_vo(vo_est)
    state = estimator.get_rover_state(current_time=100.2)

    assert state.pose.position.x > 0.1
    assert estimator.health.vo_health == 0.9


def test_state_estimator_vo_lost_fallback():
    estimator = StateEstimator(initial_timestamp=100.0)
    vo_lost = VOEstimate(
        timestamp=100.2,
        delta_position=Vector3(x=5.0, y=0.0, z=0.0),  # Bogus delta
        tracking_quality=TrackingQuality.LOST,
        camera_health=0.1,
    )
    estimator.update_vo(vo_lost)
    state = estimator.get_rover_state(current_time=100.2)

    # When VO is lost, delta_position must NOT be integrated!
    assert state.pose.position.x == 0.0
    assert estimator.health.vo_health == 0.0


def test_state_estimator_staleness_decay():
    estimator = StateEstimator(initial_timestamp=100.0)
    estimator.update_imu(timestamp=100.0, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0)
    estimator.update_wheel_odometry(timestamp=100.0, wheel_rpm=[0.0], linear_speed_mps=0.0)

    # Query 2 seconds later without telemetry updates
    state = estimator.get_rover_state(current_time=102.0)
    assert state.telemetry_age_ms >= 2000.0
    assert estimator.health.imu_health < 1.0
