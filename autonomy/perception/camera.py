"""Camera stream consumer and frame decoder."""

import time
from typing import Optional
import cv2
import numpy as np

from autonomy.perception.base import CameraFrame
from video_feed import Frame, FrameBroadcaster


class CameraIngestor:
    """Ingests, validates, decodes, and sequence-tracks frames from FrameBroadcaster."""

    def __init__(
        self,
        broadcaster: Optional[FrameBroadcaster] = None,
        view: str = "front",
        max_age_s: float = 0.15,
    ):
        self.broadcaster = broadcaster
        self.view = view
        self.max_age_s = max_age_s

        self.sequence_counter: int = 0
        self.duplicate_count: int = 0
        self.stale_drop_count: int = 0
        self._last_captured_at: Optional[float] = None
        self._last_frame: Optional[CameraFrame] = None

    def decode_frame(self, frame: Frame, current_monotonic: Optional[float] = None) -> Optional[CameraFrame]:
        """Decode raw JPEG frame into validated numpy CameraFrame."""
        now = time.monotonic() if current_monotonic is None else current_monotonic
        age_s = now - frame.captured_monotonic

        # 1. Drop stale frames
        if age_s > self.max_age_s:
            self.stale_drop_count += 1
            return None

        # 2. Check duplicate timestamp
        if self._last_captured_at is not None and frame.captured_at == self._last_captured_at:
            self.duplicate_count += 1
            return self._last_frame

        # 3. Decode JPEG to BGR numpy array
        nparr = np.frombuffer(frame.jpeg, np.uint8)
        image_bgr = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        if image_bgr is None:
            return None

        self.sequence_counter += 1
        self._last_captured_at = frame.captured_at

        camera_frame = CameraFrame(
            sequence_id=self.sequence_counter,
            timestamp=frame.captured_at,
            timestamp_monotonic=frame.captured_monotonic,
            image_bgr=image_bgr,
            age_ms=round(age_s * 1000.0, 2),
            view=self.view,
        )
        self._last_frame = camera_frame
        return camera_frame

    async def get_latest_frame(self, max_age: Optional[float] = None) -> Optional[CameraFrame]:
        """Fetch and decode latest frame from broadcaster."""
        if not self.broadcaster:
            return None

        effective_max_age = max_age or self.max_age_s
        frame = await self.broadcaster.get_frame(max_age=effective_max_age)
        if not frame:
            return None

        return self.decode_frame(frame)
