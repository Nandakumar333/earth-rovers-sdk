"""Tests for FeatureVisualOdometry and VO contracts."""

import time
import numpy as np
import pytest

from autonomy.contracts.perception import CameraHealthScore
from autonomy.localization.contracts import TrackingQuality
from autonomy.localization.vo import FeatureVisualOdometry
from autonomy.perception.base import CameraFrame


def _create_textured_frame(timestamp: float, shift_y: int = 0) -> CameraFrame:
    """Create a synthetic high-contrast textured image with a shift."""
    np.random.seed(42)
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    # Add high-contrast grid patterns
    for y in range(20, 220, 20):
        for x in range(20, 300, 20):
            pt_y = min(max(y + shift_y, 0), 239)
            img[pt_y - 4 : pt_y + 4, x - 4 : x + 4] = [255, 255, 255]

    return CameraFrame(
        sequence_id=1,
        timestamp=timestamp,
        timestamp_monotonic=timestamp,
        image_bgr=img,
        age_ms=10.0,
    )


def test_vo_initial_frame():
    vo = FeatureVisualOdometry()
    frame = _create_textured_frame(time.monotonic())
    estimate = vo.update(frame)

    assert estimate.tracking_quality == TrackingQuality.OPTIMAL
    assert estimate.feature_count > 0
    assert estimate.delta_position.x == 0.0
    assert estimate.delta_position.y == 0.0


def test_vo_tracks_forward_motion():
    vo = FeatureVisualOdometry(pixels_per_meter=100.0)
    t0 = time.monotonic()
    frame1 = _create_textured_frame(t0, shift_y=0)
    frame2 = _create_textured_frame(t0 + 0.1, shift_y=10)  # Downward optical flow = forward motion

    vo.update(frame1)
    estimate = vo.update(frame2)

    assert estimate.tracking_quality == TrackingQuality.OPTIMAL
    assert estimate.feature_count >= 8
    assert estimate.inlier_ratio >= 0.5
    assert estimate.delta_position.x > 0.0  # forward motion
    assert estimate.linear_velocity.x > 0.0


def test_vo_degrades_on_low_camera_health():
    vo = FeatureVisualOdometry()
    t0 = time.monotonic()
    frame1 = _create_textured_frame(t0)
    frame2 = _create_textured_frame(t0 + 0.1)

    vo.update(frame1)
    degraded_health = CameraHealthScore(score=0.35, is_degraded=True, blur_score=15.0)
    estimate = vo.update(frame2, camera_health=degraded_health)

    assert estimate.tracking_quality == TrackingQuality.DEGRADED
    assert estimate.covariance[0] >= 5.0


def test_vo_lost_on_severe_visual_blackout():
    vo = FeatureVisualOdometry()
    t0 = time.monotonic()
    frame1 = _create_textured_frame(t0)
    frame2 = _create_textured_frame(t0 + 0.1)

    vo.update(frame1)
    lost_health = CameraHealthScore(score=0.1, is_degraded=True, is_usable=False)
    estimate = vo.update(frame2, camera_health=lost_health)

    assert estimate.tracking_quality == TrackingQuality.LOST
    assert estimate.covariance[0] >= 10.0


def test_vo_reset():
    vo = FeatureVisualOdometry()
    frame = _create_textured_frame(time.monotonic())
    vo.update(frame)
    vo.reset()
    assert vo._prev_gray is None
    assert vo._prev_pts is None
