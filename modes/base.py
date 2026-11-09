# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Avnet
"""Strategy interface for vision modes.

Each mode owns its own model loading, per-frame inference, and arm-control
decisions. The outer loop in main.py owns the camera, display window, and
IoTConnect command pump — those are shared across all modes.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np
    import xarm


class Mode:
    name = "base"

    def setup(self, arm: xarm.Controller) -> None:
        """Called once before the camera loop starts. Load models, init state."""
        return None

    def process_frame(self, frame: np.ndarray, arm: xarm.Controller) -> np.ndarray | None:
        """Process one BGR frame. Return an annotated BGR image to display, or None to show frame as-is."""
        raise NotImplementedError

    def teardown(self, arm: xarm.Controller) -> None:
        """Called once after the loop exits. Release any mode-owned resources."""
        return None

    def get_state(self) -> str:
        """Short label for this mode's current state. Published as top-level
        `state` in /IOTCONNECT telemetry. Override in stateful modes; static
        modes can return a fixed label (e.g. 'ASL-Gesture')."""
        return self.name
