"""Tests for SpatialColorVPR place recognition."""

import numpy as np
import pytest

from autonomy.perception.base import CameraFrame
from autonomy.topology.vpr import SpatialColorVPR


def _make_frame(color_bgr: tuple[int, int, int]) -> CameraFrame:
    img = np.zeros((120, 160, 3), dtype=np.uint8)
    img[:] = color_bgr
    return CameraFrame(
        sequence_id=1,
        timestamp=100.0,
        timestamp_monotonic=100.0,
        image_bgr=img,
        age_ms=10.0,
    )


def test_spatial_color_vpr_embed_dimensions():
    vpr = SpatialColorVPR(grid_rows=2, grid_cols=2)
    frame = _make_frame((0, 255, 0))  # Green image
    emb = vpr.embed(frame)

    assert emb.shape[0] == 2 * 2 * 128
    assert pytest.approx(np.linalg.norm(emb), abs=1e-4) == 1.0


def test_spatial_color_vpr_self_similarity():
    vpr = SpatialColorVPR()
    frame = _make_frame((200, 100, 50))
    emb1 = vpr.embed(frame)
    emb2 = vpr.embed(frame)

    sim = float(np.dot(emb1, emb2))
    assert pytest.approx(sim, abs=1e-4) == 1.0


def test_spatial_color_vpr_different_scenes():
    vpr = SpatialColorVPR()
    frame_blue = _make_frame((255, 0, 0))
    frame_red = _make_frame((0, 0, 255))

    emb_blue = vpr.embed(frame_blue)
    emb_red = vpr.embed(frame_red)

    sim = float(np.dot(emb_blue, emb_red))
    assert sim < 0.5  # Distinct color scenes have low similarity


def test_spatial_color_vpr_retrieve():
    vpr = SpatialColorVPR()
    frame_a = _make_frame((0, 200, 100))
    frame_b = _make_frame((200, 20, 200))

    emb_a = vpr.embed(frame_a)
    emb_b = vpr.embed(frame_b)

    vpr.add_place("place_a", emb_a)
    vpr.add_place("place_b", emb_b)

    results = vpr.retrieve(emb_a, k=2)
    assert len(results) >= 1
    assert results[0][0] == "place_a"
    assert pytest.approx(results[0][1], abs=1e-3) == 1.0
