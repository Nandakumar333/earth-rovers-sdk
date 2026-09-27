"""Unit tests for TemporalFusion."""

import numpy as np
import pytest

from autonomy.contracts.perception import CameraHealthScore
from autonomy.perception.base import DepthEstimate
from autonomy.perception.temporal_fusion import TemporalFusion


def test_temporal_fusion_initial_frame():
    fusion = TemporalFusion(depth_alpha=0.7)
    depth_arr = np.full((10, 10), 5.0, dtype=np.float32)
    est = DepthEstimate(
        timestamp=100.0,
        frame_id="f1",
        depth_m=depth_arr,
        min_depth_m=0.2,
        max_depth_m=15.0,
    )

    filtered = fusion.filter_depth(est)
    assert np.allclose(filtered.depth_m, 5.0)
    assert fusion.observations_count == 1


def test_temporal_fusion_ema_smoothing():
    fusion = TemporalFusion(depth_alpha=0.5)
    f1 = DepthEstimate(
        timestamp=100.0,
        frame_id="f1",
        depth_m=np.full((10, 10), 10.0, dtype=np.float32),
    )
    fusion.filter_depth(f1)

    # Frame 2 has depth 6.0: with alpha=0.5, EMA = 0.5 * 6.0 + 0.5 * 10.0 = 8.0
    f2 = DepthEstimate(
        timestamp=100.1,
        frame_id="f2",
        depth_m=np.full((10, 10), 6.0, dtype=np.float32),
    )
    filtered = fusion.filter_depth(f2)
    assert np.allclose(filtered.depth_m, 8.0)


def test_temporal_fusion_health_dampening():
    fusion = TemporalFusion(health_alpha=0.5)

    # Healthy steady-state
    h1 = CameraHealthScore(score=0.9, blur_score=150.0, is_degraded=False, is_usable=True)
    fusion.filter_health(h1)

    # Momentary transient drop (e.g. 1 frame sunlight reflection)
    h2 = CameraHealthScore(score=0.3, blur_score=30.0, is_degraded=True, is_usable=True)
    filtered = fusion.filter_health(h2)

    # Blended score: 0.5 * 0.3 + 0.5 * 0.9 = 0.60 (dampens spike)
    assert filtered.score == pytest.approx(0.60, 0.05)


def test_temporal_fusion_reset():
    fusion = TemporalFusion()
    f1 = DepthEstimate(
        timestamp=100.0,
        frame_id="f1",
        depth_m=np.full((5, 5), 4.0, dtype=np.float32),
    )
    fusion.filter_depth(f1)
    assert fusion.observations_count == 1

    fusion.reset()
    assert fusion.observations_count == 0
    assert fusion._smoothed_depth is None
