"""Unit tests for VisualQualityEstimator."""

import cv2
import numpy as np
import pytest

from autonomy.perception.visual_quality import VisualQualityEstimator


@pytest.fixture
def quality_estimator():
    return VisualQualityEstimator()


def test_quality_sharp_high_contrast_image(quality_estimator):
    # Create sharp checkerboard image
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    img[::20, :] = 255
    img[:, ::20] = 255

    score = quality_estimator.evaluate(img)
    assert score.score > 0.70
    assert score.blur_score > 100.0
    assert not score.is_degraded
    assert score.is_usable


def test_quality_blurred_image(quality_estimator):
    # Create blurred image
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    img[::20, :] = 255
    blurred = cv2.GaussianBlur(img, (25, 25), 0)

    score = quality_estimator.evaluate(blurred)
    assert score.blur_score < 50.0
    assert score.is_degraded


def test_quality_completely_dark_image(quality_estimator):
    # Total darkness (e.g. night or covered camera)
    dark_img = np.zeros((100, 100, 3), dtype=np.uint8)

    score = quality_estimator.evaluate(dark_img)
    assert score.brightness == 0.0
    assert score.blur_score == 0.0
    assert score.score < 0.25
    assert not score.is_usable


def test_quality_overexposed_image(quality_estimator):
    # Pure white glare
    white_img = np.full((100, 100, 3), 255, dtype=np.uint8)

    score = quality_estimator.evaluate(white_img)
    assert score.brightness == 255.0
    assert score.contrast == 0.0
    assert score.is_degraded
