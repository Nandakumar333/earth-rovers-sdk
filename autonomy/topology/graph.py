"""Topological memory graph with spatial gating, edge cost estimation, and Dijkstra pathfinding."""

import heapq
import math
from typing import Any, Dict, List, Optional, Tuple

from autonomy.contracts.state import PoseEstimate
from autonomy.topology.contracts import TopologicalEdge, TopologicalNode


class TopologicalGraph:
    """Persistent topological-semantic graph for global spatial memory and routing."""

    def __init__(
        self,
        min_node_distance_m: float = 1.0,
        min_node_heading_deg: float = 45.0,
    ):
        self.min_node_distance_m = min_node_distance_m
        self.min_node_heading_deg = min_node_heading_deg

        self.nodes: Dict[str, TopologicalNode] = {}
        self.edges: Dict[str, TopologicalEdge] = {}
        # Adjacency list: from_node -> list of (to_node, edge_id)
        self._adj: Dict[str, List[Tuple[str, str]]] = {}
        self.active_node_id: Optional[str] = None

    def __len__(self) -> int:
        return len(self.nodes)

    def should_spawn_node(
        self,
        current_pose: PoseEstimate,
        current_heading_deg: float,
    ) -> bool:
        """Evaluate spatial gating to determine if rover has moved enough to spawn a node."""
        if not self.active_node_id or self.active_node_id not in self.nodes:
            return True

        last_node = self.nodes[self.active_node_id]
        dx = current_pose.position.x - last_node.local_metric_pose.position.x
        dy = current_pose.position.y - last_node.local_metric_pose.position.y
        dist = math.sqrt(dx * dx + dy * dy)

        dh = abs((current_heading_deg - last_node.heading_deg + 180.0) % 360.0 - 180.0)

        return dist >= self.min_node_distance_m or dh >= self.min_node_heading_deg

    def add_node(
        self,
        node: TopologicalNode,
        parent_id: Optional[str] = None,
        bidirectional: bool = True,
    ) -> TopologicalNode:
        """Register a new topological node and optionally connect to its parent."""
        self.nodes[node.node_id] = node
        if node.node_id not in self._adj:
            self._adj[node.node_id] = []

        if parent_id and parent_id in self.nodes:
            parent = self.nodes[parent_id]
            dx = node.local_metric_pose.position.x - parent.local_metric_pose.position.x
            dy = node.local_metric_pose.position.y - parent.local_metric_pose.position.y
            dist = max(math.sqrt(dx * dx + dy * dy), 0.01)
            dh = (node.heading_deg - parent.heading_deg + 180.0) % 360.0 - 180.0

            edge_fwd_id = f"{parent_id}->{node.node_id}"
            edge_fwd = TopologicalEdge(
                edge_id=edge_fwd_id,
                from_node=parent_id,
                to_node=node.node_id,
                estimated_distance_m=dist,
                estimated_travel_time_s=dist / 0.5,  # nominal 0.5 m/s
                heading_delta_deg=dh,
            )
            self.add_edge(edge_fwd, bidirectional=bidirectional)

        self.active_node_id = node.node_id
        return node

    def add_edge(self, edge: TopologicalEdge, bidirectional: bool = True) -> None:
        """Add directed or bidirectional edge between topological nodes."""
        self.edges[edge.edge_id] = edge
        if edge.from_node not in self._adj:
            self._adj[edge.from_node] = []
        self._adj[edge.from_node].append((edge.to_node, edge.edge_id))

        if edge.from_node in self.nodes:
            self.nodes[edge.from_node].neighbor_edges.append(edge.edge_id)

        if bidirectional:
            edge_rev_id = f"{edge.to_node}->{edge.from_node}"
            if edge_rev_id not in self.edges:
                edge_rev = TopologicalEdge(
                    edge_id=edge_rev_id,
                    from_node=edge.to_node,
                    to_node=edge.from_node,
                    estimated_distance_m=edge.estimated_distance_m,
                    estimated_travel_time_s=edge.estimated_travel_time_s,
                    heading_delta_deg=-edge.heading_delta_deg,
                    terrain_difficulty=edge.terrain_difficulty,
                    risk=edge.risk,
                    slip_evidence=edge.slip_evidence,
                )
                self.edges[edge_rev_id] = edge_rev
                if edge.to_node not in self._adj:
                    self._adj[edge.to_node] = []
                self._adj[edge.to_node].append((edge.from_node, edge_rev_id))
                if edge.to_node in self.nodes:
                    self.nodes[edge.to_node].neighbor_edges.append(edge_rev_id)

    def get_neighbors(self, node_id: str) -> List[Tuple[TopologicalNode, TopologicalEdge]]:
        """Retrieve neighboring nodes and connecting edges."""
        if node_id not in self._adj:
            return []
        results = []
        for target_id, edge_id in self._adj[node_id]:
            if target_id in self.nodes and edge_id in self.edges:
                results.append((self.nodes[target_id], self.edges[edge_id]))
        return results

    def find_path(
        self,
        start_node_id: str,
        goal_node_id: str,
        risk_weight: float = 2.0,
    ) -> Optional[List[str]]:
        """Compute minimum-cost topological route using Dijkstra search."""
        if start_node_id not in self.nodes or goal_node_id not in self.nodes:
            return None

        if start_node_id == goal_node_id:
            return [start_node_id]

        # Priority queue entries: (cost, current_node_id, path)
        pq: List[Tuple[float, str, List[str]]] = [(0.0, start_node_id, [start_node_id])]
        visited: Dict[str, float] = {start_node_id: 0.0}

        while pq:
            cost, curr_id, path = heapq.heappop(pq)

            if curr_id == goal_node_id:
                return path

            if cost > visited.get(curr_id, float("inf")):
                continue

            for neighbor, edge in self.get_neighbors(curr_id):
                # Composite cost = distance * (1.0 + difficulty + risk_weight * risk + slip)
                multiplier = 1.0 + edge.terrain_difficulty + (risk_weight * edge.risk) + edge.slip_evidence
                step_cost = edge.estimated_distance_m * multiplier
                next_cost = cost + step_cost

                if next_cost < visited.get(neighbor.node_id, float("inf")):
                    visited[neighbor.node_id] = next_cost
                    heapq.heappush(pq, (next_cost, neighbor.node_id, path + [neighbor.node_id]))

        return None

    def update_edge_traversal(
        self,
        edge_id: str,
        success: bool,
        slip_delta: float = 0.0,
    ) -> None:
        """Record outcome of traversing an edge to update difficulty and risk."""
        if edge_id in self.edges:
            edge = self.edges[edge_id]
            if success:
                edge.successful_traversals += 1
                edge.risk = max(0.0, edge.risk - 0.05)
            else:
                edge.failed_traversals += 1
                edge.risk = min(1.0, edge.risk + 0.2)

            if slip_delta > 0.0:
                edge.slip_evidence = min(1.0, edge.slip_evidence + slip_delta)

    def export_summary(self) -> Dict[str, Any]:
        """Generate high-level topology diagnostics for status snapshots."""
        total_dist = sum(e.estimated_distance_m for e in self.edges.values())
        avg_risk = (
            sum(e.risk for e in self.edges.values()) / max(len(self.edges), 1)
            if self.edges
            else 0.0
        )
        return {
            "node_count": len(self.nodes),
            "edge_count": len(self.edges),
            "active_node_id": self.active_node_id,
            "total_distance_m": round(total_dist, 2),
            "average_risk": round(avg_risk, 3),
        }
