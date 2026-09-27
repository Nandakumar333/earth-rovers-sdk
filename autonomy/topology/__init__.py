"""Topological memory and place recognition module."""

from autonomy.topology.contracts import (
    LoopClosureStatus,
    PlaceCandidate,
    TopologicalEdge,
    TopologicalNode,
    VerificationResult,
)
from autonomy.topology.graph import TopologicalGraph
from autonomy.topology.loop_verifier import LoopClosureVerifier
from autonomy.topology.vector_index import VectorIndex
from autonomy.topology.vpr import PlaceRecognitionProvider, SpatialColorVPR

__all__ = [
    "LoopClosureStatus",
    "PlaceCandidate",
    "VerificationResult",
    "TopologicalNode",
    "TopologicalEdge",
    "TopologicalGraph",
    "LoopClosureVerifier",
    "VectorIndex",
    "PlaceRecognitionProvider",
    "SpatialColorVPR",
]
