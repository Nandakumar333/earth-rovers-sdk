"""Tests for TopologicalGraph and Dijkstra routing."""

import pytest

from autonomy.contracts.state import PoseEstimate, Vector3
from autonomy.topology.contracts import TopologicalEdge, TopologicalNode
from autonomy.topology.graph import TopologicalGraph


def _make_node(node_id: str, x: float, y: float, heading: float = 0.0) -> TopologicalNode:
    return TopologicalNode(
        node_id=node_id,
        capture_timestamp=100.0,
        local_metric_pose=PoseEstimate(position=Vector3(x=x, y=y, z=0.0)),
        heading_deg=heading,
    )


def test_graph_spatial_gating():
    graph = TopologicalGraph(min_node_distance_m=1.0, min_node_heading_deg=45.0)

    # First node always spawns
    p0 = PoseEstimate(position=Vector3(x=0.0, y=0.0, z=0.0))
    assert graph.should_spawn_node(p0, 0.0) is True

    n0 = _make_node("n0", 0.0, 0.0, 0.0)
    graph.add_node(n0)

    # Small displacement (0.5m, same heading) -> should NOT spawn
    p_small = PoseEstimate(position=Vector3(x=0.5, y=0.0, z=0.0))
    assert graph.should_spawn_node(p_small, 0.0) is False

    # Heading turn (50 deg) -> should spawn
    assert graph.should_spawn_node(p_small, 50.0) is True

    # Distance displacement (1.2m) -> should spawn
    p_far = PoseEstimate(position=Vector3(x=1.2, y=0.0, z=0.0))
    assert graph.should_spawn_node(p_far, 0.0) is True


def test_graph_add_node_creates_bidirectional_edge():
    graph = TopologicalGraph()
    n0 = _make_node("n0", 0.0, 0.0)
    n1 = _make_node("n1", 2.0, 0.0)

    graph.add_node(n0)
    graph.add_node(n1, parent_id="n0", bidirectional=True)

    assert len(graph.nodes) == 2
    assert len(graph.edges) == 2
    assert "n0->n1" in graph.edges
    assert "n1->n0" in graph.edges

    neighbors_n0 = graph.get_neighbors("n0")
    assert len(neighbors_n0) == 1
    assert neighbors_n0[0][0].node_id == "n1"


def test_graph_dijkstra_pathfinding():
    graph = TopologicalGraph()
    n0 = _make_node("n0", 0.0, 0.0)
    n1 = _make_node("n1", 1.0, 0.0)
    n2 = _make_node("n2", 2.0, 0.0)

    graph.add_node(n0)
    graph.add_node(n1, parent_id="n0")
    graph.add_node(n2, parent_id="n1")

    path = graph.find_path("n0", "n2")
    assert path == ["n0", "n1", "n2"]


def test_graph_dijkstra_avoids_high_risk_edge():
    graph = TopologicalGraph()
    # Triangle: A -> B (direct, but dangerous), A -> C -> B (detour, but safe)
    graph.add_node(_make_node("A", 0.0, 0.0))
    graph.add_node(_make_node("B", 1.0, 0.0))
    graph.add_node(_make_node("C", 0.5, 0.5))

    # Direct edge A->B: 1.0m, risk 0.9
    edge_ab = TopologicalEdge(
        edge_id="A->B",
        from_node="A",
        to_node="B",
        estimated_distance_m=1.0,
        risk=0.9,
    )
    # Detour A->C: 0.7m, risk 0.0; C->B: 0.7m, risk 0.0
    edge_ac = TopologicalEdge(
        edge_id="A->C",
        from_node="A",
        to_node="C",
        estimated_distance_m=0.7,
        risk=0.0,
    )
    edge_cb = TopologicalEdge(
        edge_id="C->B",
        from_node="C",
        to_node="B",
        estimated_distance_m=0.7,
        risk=0.0,
    )

    graph.add_edge(edge_ab, bidirectional=False)
    graph.add_edge(edge_ac, bidirectional=False)
    graph.add_edge(edge_cb, bidirectional=False)

    # Cost AB: 1.0 * (1 + 2 * 0.9) = 2.8
    # Cost ACB: 0.7 * 1.0 + 0.7 * 1.0 = 1.4 -> Safer route should be chosen!
    path = graph.find_path("A", "B", risk_weight=2.0)
    assert path == ["A", "C", "B"]


def test_graph_update_edge_traversal():
    graph = TopologicalGraph()
    edge = TopologicalEdge(
        edge_id="e1",
        from_node="a",
        to_node="b",
        risk=0.5,
    )
    graph.add_edge(edge, bidirectional=False)

    graph.update_edge_traversal("e1", success=True, slip_delta=0.1)
    assert edge.successful_traversals == 1
    assert edge.risk == 0.45
    assert edge.slip_evidence == 0.1

    graph.update_edge_traversal("e1", success=False)
    assert edge.failed_traversals == 1
    assert edge.risk == 0.65


def test_graph_export_summary():
    graph = TopologicalGraph()
    graph.add_node(_make_node("n0", 0.0, 0.0))
    graph.add_node(_make_node("n1", 2.0, 0.0), parent_id="n0")

    summary = graph.export_summary()
    assert summary["node_count"] == 2
    assert summary["edge_count"] == 2
    assert summary["active_node_id"] == "n1"
    assert summary["total_distance_m"] >= 4.0
