"""Multi-factor loop closure verification engine."""

import math
from typing import Optional

from autonomy.contracts.state import PoseEstimate
from autonomy.topology.contracts import (
    LoopClosureStatus,
    PlaceCandidate,
    TopologicalNode,
    VerificationResult,
)


class LoopClosureVerifier:
    """Verifies loop closure candidates using multi-factor consistency checks.
    
    Guarantees no false positive topological loops by enforcing appearance similarity,
    heading compatibility, temporal context, and spatial consistency per PRD 15.3 & 15.6.
    """

    def __init__(
        self,
        min_temporal_gap_s: float = 10.0,
        min_appearance_verified: float = 0.85,
        min_appearance_candidate: float = 0.65,
        max_heading_error_deg: float = 40.0,
        allow_reverse_heading: bool = False,
        max_metric_error_m: float = 15.0,
    ):
        self.min_temporal_gap_s = min_temporal_gap_s
        self.min_appearance_verified = min_appearance_verified
        self.min_appearance_candidate = min_appearance_candidate
        self.max_heading_error_deg = max_heading_error_deg
        self.allow_reverse_heading = allow_reverse_heading
        self.max_metric_error_m = max_metric_error_m

    def verify(
        self,
        current_time: float,
        current_pose: PoseEstimate,
        current_heading_deg: float,
        candidate_node: TopologicalNode,
        appearance_similarity: float,
    ) -> VerificationResult:
        """Evaluate multi-criteria loop verification for a candidate match."""
        # 1. Temporal context check
        temporal_gap_s = current_time - candidate_node.capture_timestamp
        if temporal_gap_s < self.min_temporal_gap_s:
            return VerificationResult(
                status=LoopClosureStatus.REJECTED,
                confidence=0.0,
                appearance_score=appearance_similarity,
                temporal_gap_s=temporal_gap_s,
                details={"reason": "temporal_proximity", "min_required_s": self.min_temporal_gap_s},
            )

        # 2. Appearance threshold check
        if appearance_similarity < self.min_appearance_candidate:
            return VerificationResult(
                status=LoopClosureStatus.REJECTED,
                confidence=0.0,
                appearance_score=appearance_similarity,
                temporal_gap_s=temporal_gap_s,
                details={"reason": "low_appearance_similarity", "score": appearance_similarity},
            )

        # 3. Heading compatibility check
        heading_diff = abs(
            (current_heading_deg - candidate_node.heading_deg + 180.0) % 360.0 - 180.0
        )
        if self.allow_reverse_heading:
            # Check either forward alignment or 180-deg reverse alignment
            rev_diff = abs(heading_diff - 180.0)
            effective_heading_error = min(heading_diff, rev_diff)
        else:
            effective_heading_error = heading_diff

        if effective_heading_error > self.max_heading_error_deg:
            return VerificationResult(
                status=LoopClosureStatus.REJECTED,
                confidence=0.0,
                appearance_score=appearance_similarity,
                heading_error_deg=effective_heading_error,
                temporal_gap_s=temporal_gap_s,
                details={"reason": "heading_incompatible", "error_deg": effective_heading_error},
            )

        # 4. Metric spatial consistency check
        dx = current_pose.position.x - candidate_node.local_metric_pose.position.x
        dy = current_pose.position.y - candidate_node.local_metric_pose.position.y
        metric_dist = math.sqrt(dx * dx + dy * dy)

        heading_factor = max(0.0, 1.0 - (effective_heading_error / self.max_heading_error_deg))
        composite_confidence = 0.6 * appearance_similarity + 0.4 * heading_factor

        # Plausible metric check
        if metric_dist > self.max_metric_error_m:
            return VerificationResult(
                status=LoopClosureStatus.CANDIDATE,
                confidence=composite_confidence * 0.7,
                appearance_score=appearance_similarity,
                heading_error_deg=effective_heading_error,
                temporal_gap_s=temporal_gap_s,
                details={"reason": "metric_distance_unverified", "distance_m": metric_dist},
            )

        # 5. Final classification
        if appearance_similarity >= self.min_appearance_verified:
            return VerificationResult(
                status=LoopClosureStatus.VERIFIED,
                confidence=composite_confidence,
                appearance_score=appearance_similarity,
                heading_error_deg=effective_heading_error,
                temporal_gap_s=temporal_gap_s,
                details={"distance_m": metric_dist, "verdict": "verified_loop_closure"},
            )
        else:
            return VerificationResult(
                status=LoopClosureStatus.CANDIDATE,
                confidence=composite_confidence,
                appearance_score=appearance_similarity,
                heading_error_deg=effective_heading_error,
                temporal_gap_s=temporal_gap_s,
                details={"distance_m": metric_dist, "verdict": "requires_further_evidence"},
            )
