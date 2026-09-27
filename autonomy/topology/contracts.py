"""Contracts for topological graph, vector index, and place recognition."""

from enum import Enum
from typing import Any, Dict, List, Optional, Tuple
from pydantic import BaseModel, Field

from autonomy.contracts.state import PoseEstimate


class LoopClosureStatus(str, Enum):
    """Classification states for candidate loop closures."""

    CANDIDATE = "CANDIDATE"
    VERIFIED = "VERIFIED"
    REJECTED = "REJECTED"


class PlaceCandidate(BaseModel):
    """Candidate place match retrieved from vector index."""

    node_id: str
    similarity: float = Field(ge=-1.0, le=1.0)
    timestamp: float
    heading_deg: float
    metric_pose: PoseEstimate = Field(default_factory=PoseEstimate)


class VerificationResult(BaseModel):
    """Multi-factor verification outcome for a proposed loop closure."""

    status: LoopClosureStatus
    confidence: float = Field(default=0.0, ge=0.0, le=1.0)
    appearance_score: float = 0.0
    heading_error_deg: float = 0.0
    temporal_gap_s: float = 0.0
    geometric_inliers: int = 0
    details: Dict[str, Any] = Field(default_factory=dict)


class TopologicalNode(BaseModel):
    """Topological graph node encapsulating visual, metric, and terrain signatures."""

    node_id: str
    embedding: List[float] = Field(default_factory=list)
    capture_timestamp: float
    approximate_gps: Optional[Tuple[float, float]] = None
    local_metric_pose: PoseEstimate = Field(default_factory=PoseEstimate)
    heading_deg: float = 0.0
    terrain_signature: Dict[str, float] = Field(default_factory=dict)
    scene_quality: float = 1.0
    objects_observed: List[str] = Field(default_factory=list)
    search_coverage: float = 0.0
    risk_score: float = 0.0
    parent_edges: List[str] = Field(default_factory=list)
    neighbor_edges: List[str] = Field(default_factory=list)


class TopologicalEdge(BaseModel):
    """Directed/bidirectional edge between topological nodes with traversability cost."""

    edge_id: str
    from_node: str
    to_node: str
    estimated_distance_m: float = 0.0
    estimated_travel_time_s: float = 0.0
    heading_delta_deg: float = 0.0
    terrain_difficulty: float = Field(default=0.0, ge=0.0, le=1.0)
    risk: float = Field(default=0.0, ge=0.0, le=1.0)
    slip_evidence: float = Field(default=0.0, ge=0.0, le=1.0)
    successful_traversals: int = 0
    failed_traversals: int = 0
