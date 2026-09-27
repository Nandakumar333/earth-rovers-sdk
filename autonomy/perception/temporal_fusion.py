"""Temporal observation fusion and exponential moving average filter."""

from typing import Optional
import numpy as np

from autonomy.contracts.perception import CameraHealthScore
from autonomy.perception.base import DepthEstimate


class TemporalFusion:
    """Applies exponential moving average (EMA) smoothing and outlier dampening across frames."""

    def __init__(self, depth_alpha: float = 0.7, health_alpha: float = 0.6):
        self.depth_alpha = depth_alpha
        self.health_alpha = health_alpha

        self._smoothed_depth: Optional[np.ndarray] = None
        self._smoothed_confidence: Optional[np.ndarray] = None
        self._last_health: Optional[CameraHealthScore] = None
        self.observations_count: int = 0

    def filter_depth(self, estimate: DepthEstimate) -> DepthEstimate:
        """Blend incoming depth estimate with prior observation history."""
        incoming_depth = np.asarray(estimate.depth_m, dtype=np.float32)
        self.observations_count += 1

        if self._smoothed_depth is None or self._smoothed_depth.shape != incoming_depth.shape:
            self._smoothed_depth = incoming_depth.copy()
            if estimate.confidence_map is not None:
                self._smoothed_confidence = np.asarray(estimate.confidence_map, dtype=np.float32).copy()
        else:
            # Exponential Moving Average: Smoothed = alpha * new + (1 - alpha) * old
            self._smoothed_depth = (
                self.depth_alpha * incoming_depth
                + (1.0 - self.depth_alpha) * self._smoothed_depth
            )
            if estimate.confidence_map is not None and self._smoothed_confidence is not None:
                self._smoothed_confidence = (
                    self.depth_alpha * np.asarray(estimate.confidence_map, dtype=np.float32)
                    + (1.0 - self.depth_alpha) * self._smoothed_confidence
                )

        return DepthEstimate(
            timestamp=estimate.timestamp,
            frame_id=estimate.frame_id,
            depth_m=self._smoothed_depth.copy(),
            confidence_map=self._smoothed_confidence.copy() if self._smoothed_confidence is not None else None,
            min_depth_m=estimate.min_depth_m,
            max_depth_m=estimate.max_depth_m,
        )

    def filter_health(self, health: CameraHealthScore) -> CameraHealthScore:
        """Dampen single-frame transient illumination spikes or momentary lens glare."""
        if self._last_health is None:
            self._last_health = health
            return health

        # Blend composite score to avoid single-frame spurious emergency stops
        smoothed_score = (
            self.health_alpha * health.score
            + (1.0 - self.health_alpha) * self._last_health.score
        )

        filtered = CameraHealthScore(
            score=round(smoothed_score, 3),
            blur_score=health.blur_score,
            brightness=health.brightness,
            contrast=health.contrast,
            is_degraded=smoothed_score < 0.65 or health.is_degraded,
            is_usable=smoothed_score >= 0.25 and health.is_usable,
        )
        self._last_health = filtered
        return filtered

    def reset(self) -> None:
        """Reset temporal filter history."""
        self._smoothed_depth = None
        self._smoothed_confidence = None
        self._last_health = None
        self.observations_count = 0
