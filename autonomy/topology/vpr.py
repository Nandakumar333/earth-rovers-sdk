"""Visual Place Recognition (VPR) interface and spatial feature extractor."""

from typing import List, Optional, Protocol, Tuple
import cv2
import numpy as np

from autonomy.perception.base import CameraFrame
from autonomy.topology.contracts import (
    LoopClosureStatus,
    PlaceCandidate,
    VerificationResult,
)
from autonomy.topology.vector_index import VectorIndex


class PlaceRecognitionProvider(Protocol):
    """Protocol defining place recognition embedding and candidate retrieval."""

    def embed(self, frame: CameraFrame) -> np.ndarray:
        """Extract a normalized embedding vector from a camera frame."""
        ...

    def retrieve(
        self, embedding: np.ndarray, k: int = 5, min_similarity: float = 0.5
    ) -> List[Tuple[str, float]]:
        """Retrieve candidate node IDs and similarity scores."""
        ...


class SpatialColorVPR:
    """Lightweight edge VPR extractor combining spatial multi-bin color and edge histograms.
    
    Robust to moderate illumination changes and viewpoint jitter without requiring
    heavy external deep learning model checkpoints on edge rover compute.
    """

    def __init__(
        self,
        grid_rows: int = 2,
        grid_cols: int = 2,
        vector_index: Optional[VectorIndex] = None,
    ):
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols
        # 8 hue bins x 4 saturation bins x 4 value bins = 128 bins per grid cell
        self.bins_per_cell = 128
        self.embedding_dim = self.grid_rows * self.grid_cols * self.bins_per_cell
        self.index = vector_index or VectorIndex(dimension=self.embedding_dim)

    def embed(self, frame: CameraFrame) -> np.ndarray:
        """Generate a normalized 1D descriptor vector for the frame."""
        img = frame.image_bgr
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, w = hsv.shape[:2]

        cell_h = h // self.grid_rows
        cell_w = w // self.grid_cols

        features = []
        for r in range(self.grid_rows):
            for c in range(self.grid_cols):
                y0 = r * cell_h
                y1 = (r + 1) * cell_h if r < self.grid_rows - 1 else h
                x0 = c * cell_w
                x1 = (c + 1) * cell_w if c < self.grid_cols - 1 else w

                cell = hsv[y0:y1, x0:x1]
                # 3D Joint Hue-Saturation-Value histogram (8x4x4 = 128 bins)
                hist_3d = cv2.calcHist(
                    [cell], [0, 1, 2], None, [8, 4, 4], [0, 180, 0, 256, 0, 256]
                ).flatten()

                norm = np.linalg.norm(hist_3d)
                if norm > 1e-6:
                    hist_3d = hist_3d / norm
                features.append(hist_3d)

        descriptor = np.concatenate(features).astype(np.float32)
        norm = np.linalg.norm(descriptor)
        if norm > 1e-6:
            descriptor = descriptor / norm
        return descriptor

    def add_place(self, node_id: str, embedding: np.ndarray) -> None:
        """Register a node embedding into the vector index."""
        self.index.add(node_id, embedding)

    def retrieve(
        self, embedding: np.ndarray, k: int = 5, min_similarity: float = 0.5
    ) -> List[Tuple[str, float]]:
        """Query top-k nearest matching place candidates."""
        return self.index.search(embedding, top_k=k, min_similarity=min_similarity)
