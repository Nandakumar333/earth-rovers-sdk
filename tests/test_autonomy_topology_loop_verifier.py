"""Tests for LoopClosureVerifier multi-criteria verification."""

import pytest

from autonomy.contracts.state import PoseEstimate, Vector3
from autonomy.topology.contracts import LoopClosureStatus, TopologicalNode
from autonomy.topology.loop_verifier import LoopClosureVerifier


def _make_candidate(timestamp: float, x: float, y: float, heading: float) -> TopologicalNode:
    return TopologicalNode(
        node_id="cand_1",
        capture_timestamp=timestamp,
        local_metric_pose=PoseEstimate(position=Vector3(x=x, y=y, z=0.0)),
        heading_deg=heading,
    )


def test_loop_verifier_rejects_temporal_proximity():
    verifier = LoopClosureVerifier(min_temporal_gap_s=10.0)
    current_time = 105.0
    candidate = _make_candidate(timestamp=102.0, x=0.0, y=0.0, heading=0.0)  # Gap only 3s
    curr_pose = PoseEstimate(position=Vector3(x=0.0, y=0.0, z=0.0))

    result = verifier.verify(
        current_time=current_time,
        current_pose=curr_pose,
        current_heading_deg=0.0,
        candidate_node=candidate,
        appearance_similarity=0.95,
    )

    assert result.status == LoopClosureStatus.REJECTED
    assert result.details["reason"] == "temporal_proximity"


def test_loop_verifier_rejects_low_similarity():
    verifier = LoopClosureVerifier(min_appearance_candidate=0.65)
    current_time = 200.0
    candidate = _make_candidate(timestamp=100.0, x=0.0, y=0.0, heading=0.0)
    curr_pose = PoseEstimate(position=Vector3(x=0.0, y=0.0, z=0.0))

    result = verifier.verify(
        current_time=current_time,
        current_pose=curr_pose,
        current_heading_deg=0.0,
        candidate_node=candidate,
        appearance_similarity=0.50,  # Below candidate threshold
    )

    assert result.status == LoopClosureStatus.REJECTED
    assert result.details["reason"] == "low_appearance_similarity"


def test_loop_verifier_rejects_incompatible_heading():
    verifier = LoopClosureVerifier(max_heading_error_deg=40.0)
    current_time = 200.0
    candidate = _make_candidate(timestamp=100.0, x=0.0, y=0.0, heading=0.0)
    curr_pose = PoseEstimate(position=Vector3(x=0.0, y=0.0, z=0.0))

    result = verifier.verify(
        current_time=current_time,
        current_pose=curr_pose,
        current_heading_deg=90.0,  # Perpendicular heading!
        candidate_node=candidate,
        appearance_similarity=0.90,
    )

    assert result.status == LoopClosureStatus.REJECTED
    assert result.details["reason"] == "heading_incompatible"


def test_loop_verifier_candidate_on_moderate_similarity():
    verifier = LoopClosureVerifier(
        min_appearance_verified=0.85,
        min_appearance_candidate=0.65,
    )
    current_time = 200.0
    candidate = _make_candidate(timestamp=100.0, x=0.0, y=0.0, heading=10.0)
    curr_pose = PoseEstimate(position=Vector3(x=0.5, y=0.0, z=0.0))

    result = verifier.verify(
        current_time=current_time,
        current_pose=curr_pose,
        current_heading_deg=15.0,
        candidate_node=candidate,
        appearance_similarity=0.75,  # Moderate similarity
    )

    assert result.status == LoopClosureStatus.CANDIDATE
    assert result.confidence > 0.6


def test_loop_verifier_verified_on_strong_match():
    verifier = LoopClosureVerifier(
        min_appearance_verified=0.85,
        max_heading_error_deg=40.0,
    )
    current_time = 200.0
    candidate = _make_candidate(timestamp=100.0, x=1.0, y=0.0, heading=20.0)
    curr_pose = PoseEstimate(position=Vector3(x=1.2, y=0.1, z=0.0))

    result = verifier.verify(
        current_time=current_time,
        current_pose=curr_pose,
        current_heading_deg=25.0,  # Heading diff 5 deg
        candidate_node=candidate,
        appearance_similarity=0.92,
    )

    assert result.status == LoopClosureStatus.VERIFIED
    assert result.confidence > 0.8
    assert result.details["verdict"] == "verified_loop_closure"
