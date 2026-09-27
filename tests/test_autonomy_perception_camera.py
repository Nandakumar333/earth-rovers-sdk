"""Unit tests for CameraIngestor and frame decoding."""

import time
import cv2
import numpy as np
import pytest

from autonomy.perception.camera import CameraIngestor
from video_feed import Frame


@pytest.fixture
def sample_jpeg_frame():
    # Create synthetic test image (100x100 RGB checkerboard)
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    img[::20, :] = 255
    _, jpeg = cv2.imencode(".jpg", img)
    return jpeg.tobytes()


def test_camera_ingestor_decode_success(sample_jpeg_frame):
    ingestor = CameraIngestor(max_age_s=0.2)
    now = time.time()
    monotonic_now = time.monotonic()

    raw_frame = Frame(
        data_url="data:image/jpeg;base64,xyz",
        jpeg=sample_jpeg_frame,
        captured_at=now,
        captured_monotonic=monotonic_now,
    )

    decoded = ingestor.decode_frame(raw_frame, current_monotonic=monotonic_now + 0.05)
    assert decoded is not None
    assert decoded.sequence_id == 1
    assert decoded.height == 100
    assert decoded.width == 100
    assert decoded.timestamp == now
    assert decoded.age_ms == pytest.approx(50.0, 5.0)


def test_camera_ingestor_drops_stale_frame(sample_jpeg_frame):
    ingestor = CameraIngestor(max_age_s=0.1)  # max 100ms
    now = time.time()
    monotonic_now = time.monotonic()

    # Frame is 200ms old
    stale_frame = Frame(
        data_url="data:image/jpeg;base64,xyz",
        jpeg=sample_jpeg_frame,
        captured_at=now - 0.2,
        captured_monotonic=monotonic_now - 0.2,
    )

    decoded = ingestor.decode_frame(stale_frame, current_monotonic=monotonic_now)
    assert decoded is None
    assert ingestor.stale_drop_count == 1


def test_camera_ingestor_tracks_duplicates(sample_jpeg_frame):
    ingestor = CameraIngestor(max_age_s=0.5)
    now = time.time()
    monotonic_now = time.monotonic()

    frame = Frame(
        data_url="data:image/jpeg;base64,xyz",
        jpeg=sample_jpeg_frame,
        captured_at=now,
        captured_monotonic=monotonic_now,
    )

    # First decode
    f1 = ingestor.decode_frame(frame, current_monotonic=monotonic_now)
    assert f1 is not None
    assert ingestor.sequence_counter == 1

    # Second decode of identical capture timestamp
    f2 = ingestor.decode_frame(frame, current_monotonic=monotonic_now + 0.01)
    assert f2 is f1
    assert ingestor.duplicate_count == 1
    assert ingestor.sequence_counter == 1
