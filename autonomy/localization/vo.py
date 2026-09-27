"""Visual Odometry interface and feature-based optical flow tracking."""

import math
from typing import Optional, Protocol, Tuple
import cv2
import numpy as np

from autonomy.contracts.perception import CameraHealthScore
from autonomy.contracts.state import Vector3
from autonomy.localization.contracts import TrackingQuality, VOEstimate
from autonomy.perception.base import CameraFrame


class VisualOdometryProvider(Protocol):
    """Protocol for visual odometry motion estimation."""

    def update(
        self,
        frame: CameraFrame,
        camera_health: Optional[CameraHealthScore] = None,
    ) -> VOEstimate:
        """Process a camera frame and return the motion estimate delta."""
        ...

    def reset(self) -> None:
        """Reset internal tracking state."""
        ...


class FeatureVisualOdometry:
    """Optical flow visual odometry using Shi-Tomasi corners and Lucas-Kanade tracking.
    
    Robustly estimates planar motion and detects visual degradation.
    """

    def __init__(
        self,
        max_corners: int = 150,
        quality_level: float = 0.03,
        min_distance: float = 10.0,
        pixels_per_meter: float = 400.0,
        min_inlier_ratio: float = 0.45,
        min_features: int = 8,
    ):
        self.max_corners = max_corners
        self.quality_level = quality_level
        self.min_distance = min_distance
        self.pixels_per_meter = pixels_per_meter
        self.min_inlier_ratio = min_inlier_ratio
        self.min_features = min_features

        self._prev_gray: Optional[np.ndarray] = None
        self._prev_pts: Optional[np.ndarray] = None
        self._prev_timestamp: Optional[float] = None

    def reset(self) -> None:
        """Reset internal frame history."""
        self._prev_gray = None
        self._prev_pts = None
        self._prev_timestamp = None

    def update(
        self,
        frame: CameraFrame,
        camera_health: Optional[CameraHealthScore] = None,
    ) -> VOEstimate:
        """Track features between current and previous frame, returning VOEstimate."""
        current_time = frame.timestamp
        health_score = camera_health.score if camera_health is not None else 1.0
        is_degraded = (
            camera_health.is_degraded
            if camera_health is not None
            else False
        ) or health_score < 0.4

        # Convert to grayscale
        if len(frame.image_bgr.shape) == 3 and frame.image_bgr.shape[2] == 3:
            gray = cv2.cvtColor(frame.image_bgr, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame.image_bgr.copy()

        # If camera health is severely compromised, fail soft per PRD 14.4
        if health_score < 0.2:
            return VOEstimate(
                timestamp=current_time,
                tracking_quality=TrackingQuality.LOST,
                camera_health=health_score,
                covariance=[10.0] * 6,
            )

        if is_degraded:
            # Degraded visual quality -> report DEGRADED tracking rather than stale valid pose
            return VOEstimate(
                timestamp=current_time,
                tracking_quality=TrackingQuality.DEGRADED,
                camera_health=health_score,
                covariance=[5.0] * 6,
            )

        # First frame initialization
        if self._prev_gray is None or self._prev_pts is None or self._prev_timestamp is None:
            self._prev_gray = gray
            self._prev_pts = cv2.goodFeaturesToTrack(
                gray,
                maxCorners=self.max_corners,
                qualityLevel=self.quality_level,
                minDistance=self.min_distance,
            )
            self._prev_timestamp = current_time
            return VOEstimate(
                timestamp=current_time,
                tracking_quality=TrackingQuality.OPTIMAL,
                feature_count=len(self._prev_pts) if self._prev_pts is not None else 0,
                camera_health=health_score,
            )

        dt = max(current_time - self._prev_timestamp, 1e-4)

        # Track features using Lucas-Kanade optical flow
        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray,
            gray,
            self._prev_pts,
            None,
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )

        if curr_pts is None or status is None:
            self._reseed(gray, current_time)
            return VOEstimate(
                timestamp=current_time,
                tracking_quality=TrackingQuality.LOST,
                camera_health=health_score,
                covariance=[10.0] * 6,
            )

        good_prev = self._prev_pts[status == 1]
        good_curr = curr_pts[status == 1]
        feature_count = len(good_curr)

        if feature_count < self.min_features:
            self._reseed(gray, current_time)
            return VOEstimate(
                timestamp=current_time,
                feature_count=feature_count,
                tracking_quality=TrackingQuality.LOST,
                camera_health=health_score,
                covariance=[10.0] * 6,
            )

        # Compute optical flow displacement vectors: (dx, dy)
        displacements = good_curr - good_prev
        dx_pixels = displacements[:, 0]
        dy_pixels = displacements[:, 1]

        # Calculate median displacement and inliers using RANSAC-like MAD filtering
        med_dx = float(np.median(dx_pixels))
        med_dy = float(np.median(dy_pixels))

        residuals = np.sqrt((dx_pixels - med_dx) ** 2 + (dy_pixels - med_dy) ** 2)
        inlier_mask = residuals < 8.0  # pixels
        inlier_count = int(np.sum(inlier_mask))
        inlier_ratio = float(inlier_count / max(feature_count, 1))

        if inlier_ratio < self.min_inlier_ratio:
            self._reseed(gray, current_time)
            return VOEstimate(
                timestamp=current_time,
                feature_count=feature_count,
                inlier_ratio=inlier_ratio,
                tracking_quality=TrackingQuality.DEGRADED,
                camera_health=health_score,
                covariance=[5.0] * 6,
            )

        # Inlier displacements
        inlier_dx = float(np.mean(dx_pixels[inlier_mask]))
        inlier_dy = float(np.mean(dy_pixels[inlier_mask]))

        # Conversion to metric rover frame:
        # Camera is forward-facing:
        # - Vertical downward flow (dy > 0) corresponds to forward motion (+x)
        # - Horizontal flow (dx) corresponds to yaw rotation and lateral translation
        delta_x = inlier_dy / self.pixels_per_meter
        delta_y = -inlier_dx / self.pixels_per_meter
        # Approximate delta yaw from differential horizontal flow across x coordinates
        h_center = gray.shape[1] / 2.0
        x_coords = good_curr[inlier_mask][:, 0]
        left_mask = x_coords < h_center
        right_mask = x_coords >= h_center

        delta_yaw = 0.0
        if np.sum(left_mask) > 3 and np.sum(right_mask) > 3:
            left_dy = np.mean(dy_pixels[inlier_mask][left_mask])
            right_dy = np.mean(dy_pixels[inlier_mask][right_mask])
            delta_yaw = float((left_dy - right_dy) / (self.pixels_per_meter * 0.3))

        vx = delta_x / dt
        vy = delta_y / dt
        wz = delta_yaw / dt

        base_cov = 0.02 / max(inlier_ratio * health_score, 0.1)
        covariance = [base_cov] * 6

        # Update tracking history; reseed if feature count is dwindling
        if feature_count < self.max_corners // 2:
            self._prev_pts = cv2.goodFeaturesToTrack(
                gray,
                maxCorners=self.max_corners,
                qualityLevel=self.quality_level,
                minDistance=self.min_distance,
            )
        else:
            self._prev_pts = good_curr.reshape(-1, 1, 2)

        self._prev_gray = gray
        self._prev_timestamp = current_time

        return VOEstimate(
            timestamp=current_time,
            delta_position=Vector3(x=delta_x, y=delta_y, z=0.0),
            delta_yaw_rad=delta_yaw,
            linear_velocity=Vector3(x=vx, y=vy, z=0.0),
            angular_velocity=Vector3(x=0.0, y=0.0, z=wz),
            feature_count=feature_count,
            inlier_ratio=inlier_ratio,
            tracking_quality=TrackingQuality.OPTIMAL,
            covariance=covariance,
            camera_health=health_score,
        )

    def _reseed(self, gray: np.ndarray, timestamp: float) -> None:
        """Reseed features when tracking is lost or degraded."""
        self._prev_gray = gray
        self._prev_pts = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.max_corners,
            qualityLevel=self.quality_level,
            minDistance=self.min_distance,
        )
        self._prev_timestamp = timestamp
