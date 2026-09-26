"""Unit tests for autonomy canonical data contracts."""

import time
import pytest
from pydantic import ValidationError

from autonomy.contracts import (
    AutonomyState,
    CameraHealthScore,
    MissionObjective,
    MotionCommand,
    PerceptionObservation,
    PoseEstimate,
    RoverState,
    SearchTarget,
    TerrainCell,
    Vector3,
)


def test_motion_command_validation():
    now = time.time()
    cmd = MotionCommand(
        linear=0.5,
        angular=-0.3,
        valid_from=now,
        expires_at=now + 1.0,
        source="test",
        priority=50,
    )
    assert cmd.linear == 0.5
    assert cmd.angular == -0.3
    assert not cmd.is_expired(now + 0.5)
    assert cmd.is_expired(now + 1.5)
    assert cmd.is_moving()

    # Out of bounds linear
    with pytest.raises(ValidationError):
        MotionCommand(linear=1.5, angular=0.0, valid_from=now, expires_at=now + 1.0)

    # Out of bounds angular
    with pytest.raises(ValidationError):
        MotionCommand(linear=0.0, angular=-2.0, valid_from=now, expires_at=now + 1.0)


def test_motion_command_create_stop():
    cmd = MotionCommand.create_stop(priority=1, source="arbiter", ttl_s=2.0)
    assert cmd.linear == 0.0
    assert cmd.angular == 0.0
    assert not cmd.is_moving()
    assert cmd.priority == 1
    assert cmd.source == "arbiter"
    assert not cmd.is_expired()


def test_rover_state_serialization():
    now = time.time()
    state = RoverState(
        timestamp=now,
        pose=PoseEstimate(position=Vector3(x=1.0, y=2.0, z=0.5)),
        roll_deg=3.5,
        pitch_deg=-2.1,
        yaw_deg=45.0,
        wheel_rpm=[120.0, 122.0, 119.0, 121.0],
        slip_ratio=0.05,
        state_confidence=0.98,
    )

    data = state.model_dump()
    assert data["timestamp"] == now
    assert data["pose"]["position"]["x"] == 1.0
    assert data["roll_deg"] == 3.5
    assert data["wheel_rpm"] == [120.0, 122.0, 119.0, 121.0]

    # JSON roundtrip
    json_str = state.model_dump_json()
    restored = RoverState.model_validate_json(json_str)
    assert restored.pose.position.x == 1.0
    assert restored.roll_deg == 3.5


def test_perception_contracts():
    obs = PerceptionObservation(
        timestamp=time.time(),
        sensor_id="front_camera",
        frame_id="frame_001",
        confidence=0.95,
        quality=0.88,
        uncertainty={"depth_variance": 0.15},
    )
    assert obs.sensor_id == "front_camera"
    assert obs.confidence == 0.95

    health = CameraHealthScore(
        score=0.82,
        blur_score=150.0,
        brightness=120.0,
        is_degraded=False,
        is_usable=True,
    )
    assert health.score == 0.82
    assert health.is_usable


def test_terrain_cell_contract():
    cell = TerrainCell(
        x=2.5,
        y=1.0,
        elevation_m=0.3,
        slope_deg=8.5,
        obstacle_cost=0.1,
        traversability=0.85,
    )
    assert cell.x == 2.5
    assert cell.slope_deg == 8.5
    assert cell.traversability == 0.85

    with pytest.raises(ValidationError):
        TerrainCell(x=0.0, y=0.0, traversability=1.5)  # > 1.0


def test_mission_contracts():
    target = SearchTarget(
        target_id="target_canister",
        prompt="red plastic canister",
        min_confidence=0.85,
    )
    assert target.prompt == "red plastic canister"
    assert target.min_confidence == 0.85

    mission = MissionObjective(
        mission_id="mission_alpha",
        target=target,
        search_radius_m=30.0,
    )
    assert mission.mission_id == "mission_alpha"
    assert mission.target.target_id == "target_canister"
