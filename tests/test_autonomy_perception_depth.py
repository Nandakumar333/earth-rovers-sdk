"""Unit tests for DepthProvider and DepthAnythingAdapter."""

import numpy as np
import pytest

from autonomy.perception.base import DepthProvider
from autonomy.providers.depth_anything import DepthAnythingAdapter


def test_depth_anything_adapter_protocol_conformance():
    adapter = DepthAnythingAdapter()
    assert isinstance(adapter, DepthProvider)


def test_depth_estimation_geometry_and_bounds():
    adapter = DepthAnythingAdapter(min_depth_m=0.3, max_depth_m=12.0)
    # 240x320 synthetic BGR image
    img = np.full((240, 320, 3), 128, dtype=np.uint8)

    estimate = adapter.estimate_depth(img, frame_id="frame_042")
    assert estimate.frame_id == "frame_042"
    assert estimate.depth_m.shape == (240, 320)
    assert estimate.min_depth_m == 0.3
    assert estimate.max_depth_m == 12.0

    # Physical bounds check
    assert np.all(estimate.depth_m >= 0.3)
    assert np.all(estimate.depth_m <= 12.0)

    # Perspective geometry: lower rows (near ground) are closer than upper rows (horizon)
    top_row_depth = float(np.mean(estimate.depth_m[0, :]))
    bottom_row_depth = float(np.mean(estimate.depth_m[-1, :]))
    assert bottom_row_depth < top_row_depth
    assert bottom_row_depth < 1.0
    assert top_row_depth > 10.0


def test_depth_empty_image_error():
    adapter = DepthAnythingAdapter()
    with pytest.raises(ValueError):
        adapter.estimate_depth(np.array([]))
