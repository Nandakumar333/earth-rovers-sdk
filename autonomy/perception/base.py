"""Base contracts and protocols for the perception subsystem."""

from dataclasses import dataclass
from typing import Any, Optional, Protocol, runtime_checkable
import numpy as np
from pydantic import BaseModel, ConfigDict, Field


@dataclass
class CameraFrame:
    """Decoded camera frame with monotonic timestamp and sequence metadata."""

    sequence_id: int
    timestamp: float
    timestamp_monotonic: float
    image_bgr: np.ndarray
    age_ms: float
    view: str = "front"

    @property
    def height(self) -> int:
        return self.image_bgr.shape[0]

    @property
    def width(self) -> int:
        return self.image_bgr.shape[1]


class DepthEstimate(BaseModel):
    """Estimated metric depth map with confidence and physical bounds."""

    model_config = ConfigDict(arbitrary_types_allowed=True)

    timestamp: float
    frame_id: str
    depth_m: Any  # numpy.ndarray of float32
    confidence_map: Optional[Any] = None  # numpy.ndarray of float32 in [0, 1]
    min_depth_m: float = Field(default=0.2, ge=0.0)
    max_depth_m: float = Field(default=15.0, ge=0.0)


@runtime_checkable
class DepthProvider(Protocol):
    """Protocol for modular depth estimation backends."""

    def estimate_depth(self, image_bgr: np.ndarray) -> DepthEstimate:
        """Compute estimated metric depth map from input BGR camera image."""
        ...
