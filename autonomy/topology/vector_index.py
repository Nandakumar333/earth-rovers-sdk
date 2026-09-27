"""Fast in-memory normalized cosine vector index for visual place retrieval."""

from typing import List, Optional, Tuple
import numpy as np


class VectorIndex:
    """In-memory vector index supporting exact cosine k-NN search over L2-normalized embeddings."""

    def __init__(self, dimension: Optional[int] = None):
        self.dimension = dimension
        self._keys: List[str] = []
        self._matrix: Optional[np.ndarray] = None  # shape: (N, D)

    def __len__(self) -> int:
        return len(self._keys)

    def add(self, key: str, vector: List[float] | np.ndarray) -> None:
        """Add an embedding vector to the index with L2 normalization."""
        v = np.asarray(vector, dtype=np.float32).flatten()
        norm = np.linalg.norm(v)
        if norm > 1e-8:
            v = v / norm
        else:
            v = np.zeros_like(v)

        if self.dimension is None:
            self.dimension = v.shape[0]
        elif v.shape[0] != self.dimension:
            raise ValueError(
                f"Vector dimension mismatch: expected {self.dimension}, got {v.shape[0]}"
            )

        if key in self._keys:
            idx = self._keys.index(key)
            self._matrix[idx] = v
            return

        self._keys.append(key)
        if self._matrix is None:
            self._matrix = v.reshape(1, -1)
        else:
            self._matrix = np.vstack([self._matrix, v.reshape(1, -1)])

    def remove(self, key: str) -> bool:
        """Remove a key from the index."""
        if key not in self._keys:
            return False
        idx = self._keys.index(key)
        self._keys.pop(idx)
        if len(self._keys) == 0:
            self._matrix = None
        else:
            self._matrix = np.delete(self._matrix, idx, axis=0)
        return True

    def get(self, key: str) -> Optional[np.ndarray]:
        """Retrieve stored normalized vector by key."""
        if key not in self._keys or self._matrix is None:
            return None
        idx = self._keys.index(key)
        return self._matrix[idx].copy()

    def search(
        self,
        query: List[float] | np.ndarray,
        top_k: int = 5,
        min_similarity: float = -1.0,
    ) -> List[Tuple[str, float]]:
        """Search for top_k nearest neighbors by cosine similarity."""
        if self._matrix is None or len(self._keys) == 0:
            return []

        q = np.asarray(query, dtype=np.float32).flatten()
        norm = np.linalg.norm(q)
        if norm > 1e-8:
            q = q / norm
        else:
            return []

        # Cosine similarity is the dot product of L2-normalized vectors
        scores = np.dot(self._matrix, q)  # shape: (N,)

        # Sort descending
        ranked_indices = np.argsort(-scores)

        results: List[Tuple[str, float]] = []
        for idx in ranked_indices[:top_k]:
            score = float(scores[idx])
            if score >= min_similarity:
                results.append((self._keys[idx], score))

        return results

    def clear(self) -> None:
        """Clear all indexed vectors."""
        self._keys.clear()
        self._matrix = None
