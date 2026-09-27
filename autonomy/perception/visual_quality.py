"""Real-time camera frame health and visual degradation estimator."""

import cv2
import numpy as np
from autonomy.contracts.perception import CameraHealthScore


class VisualQualityEstimator:
    """Computes blur, illumination, and contrast quality metrics on camera frames."""

    def __init__(
        self,
        min_blur_variance: float = 80.0,
        min_contrast: float = 20.0,
        min_brightness: float = 25.0,
        max_brightness: float = 230.0,
    ):
        self.min_blur_variance = min_blur_variance
        self.min_contrast = min_contrast
        self.min_brightness = min_brightness
        self.max_brightness = max_brightness

    def evaluate(self, image_bgr: np.ndarray) -> CameraHealthScore:
        """Analyze frame and produce structured CameraHealthScore."""
        if image_bgr is None or image_bgr.size == 0:
            return CameraHealthScore(
                score=0.0,
                blur_score=0.0,
                brightness=0.0,
                contrast=0.0,
                is_degraded=True,
                is_usable=False,
            )

        # Convert to grayscale for illumination and Laplacian variance
        if len(image_bgr.shape) == 3:
            gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        else:
            gray = image_bgr

        # 1. Blur metric (Laplacian variance)
        laplacian = cv2.Laplacian(gray, cv2.CV_64F)
        blur_var = float(laplacian.var())

        # 2. Illumination metrics
        brightness = float(np.mean(gray))
        contrast = float(np.std(gray))

        # Normalized component scores in [0, 1]
        # Blur subscore: 1.0 when >= 200, 0.0 when <= 10
        blur_subscore = min(max((blur_var - 10.0) / (200.0 - 10.0), 0.0), 1.0)

        # Brightness subscore: penalty if too dark (< 30) or washed out (> 220)
        if brightness < self.min_brightness:
            brightness_subscore = max(brightness / self.min_brightness, 0.0)
        elif brightness > self.max_brightness:
            brightness_subscore = max((255.0 - brightness) / (255.0 - self.max_brightness), 0.0)
        else:
            brightness_subscore = 1.0

        # Contrast subscore: 1.0 when >= 40, 0.0 when <= 5
        contrast_subscore = min(max((contrast - 5.0) / 35.0, 0.0), 1.0)

        # Weighted composite score
        composite = (0.45 * blur_subscore) + (0.35 * brightness_subscore) + (0.20 * contrast_subscore)
        composite = max(0.0, min(composite, 1.0))

        is_degraded = composite < 0.65 or blur_var < self.min_blur_variance or contrast < self.min_contrast
        is_usable = composite >= 0.25 and blur_var >= 15.0 and brightness >= 10.0

        return CameraHealthScore(
            score=round(composite, 3),
            blur_score=round(blur_var, 1),
            brightness=round(brightness, 1),
            contrast=round(contrast, 1),
            is_degraded=is_degraded,
            is_usable=is_usable,
        )
