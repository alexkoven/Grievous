#!/usr/bin/env python

# Copyright 2025 Alexander Nettekoven, The University of Texas at Austin
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Grievous: XLerobot with follower arms, mobile base, head, and cameras.

Architecture:
- XLerobot provides: follower arms (2x SO-101), mobile base, head, cameras
- Leader arms are provided separately via teleoperator (e.g., bi_so100_leader)
"""

import logging
from functools import cached_property
from typing import Any

from lerobot.robots.xlerobot import XLerobot
from lerobot.robots.xlerobot.config_xlerobot import XLerobotConfig

from ..robot import Robot
from .config_grievous import GrievousConfig

logger = logging.getLogger(__name__)


class Grievous(Robot):
    """Grievous robot: XLerobot with follower arms, mobile base, head, and cameras.
    
    This robot provides:
    - 2 follower arms (SO-101)
    - Mobile base
    - Head
    - 3 cameras
    
    Leader arms are provided separately via teleoperator (e.g., bi_so100_leader).
    """

    config_class = GrievousConfig
    name = "grievous"

    def __init__(self, config: GrievousConfig):
        super().__init__(config)
        self.config = config

        # Create XLerobot configuration (follower arms + base + head + cameras)
        xlerobot_config = XLerobotConfig(
            id=f"{config.id}_xlerobot" if config.id else None,
            calibration_dir=config.calibration_dir,
            port1=config.port1,
            port2=config.port2,
            disable_torque_on_disconnect=config.disable_torque_on_disconnect,
            max_relative_target=config.max_relative_target,
            use_degrees=config.use_degrees,
            cameras=config.cameras,
            teleop_keys=config.teleop_keys,
        )

        # Instantiate XLerobot
        self.xlerobot = XLerobot(xlerobot_config)

        logger.info(f"Grievous robot initialized: {config.id}")

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        """Observation features from XLerobot (arms, base, head, cameras).
        
        Returns:
            Dictionary mapping feature names to types/shapes
        """
        return self.xlerobot.observation_features

    @cached_property
    def action_features(self) -> dict[str, type]:
        """Action features from XLerobot (arms, base, head).
        
        Returns:
            Dictionary mapping action names to types
        """
        return self.xlerobot.action_features

    @property
    def is_connected(self) -> bool:
        """Check if XLerobot is connected."""
        return self.xlerobot.is_connected

    @property
    def is_calibrated(self) -> bool:
        """Check if XLerobot is calibrated."""
        return self.xlerobot.is_calibrated

    def connect(self, calibrate: bool = True) -> None:
        """Connect XLerobot.
        
        Args:
            calibrate: Whether to calibrate after connecting
        """
        logger.info("Connecting Grievous...")
        self.xlerobot.connect(calibrate=calibrate)
        logger.info("Grievous connected successfully")

    def disconnect(self) -> None:
        """Disconnect XLerobot."""
        logger.info("Disconnecting Grievous...")
        self.xlerobot.disconnect()
        logger.info("Grievous disconnected")

    def calibrate(self) -> None:
        """Calibrate XLerobot."""
        logger.info("Calibrating Grievous...")
        self.xlerobot.calibrate()
        logger.info("Grievous calibrated successfully")

    def configure(self) -> None:
        """Configure XLerobot.
        
        Sets motor parameters, control modes, and initial state.
        """
        logger.info("Configuring Grievous...")
        self.xlerobot.configure()
        logger.info("Grievous configured successfully")

    def get_observation(self) -> dict[str, Any]:
        """Get observation from XLerobot (arms, base, head, cameras).
        
        Returns:
            Dictionary with observation data
        """
        return self.xlerobot.get_observation()

    def send_action(self, action: dict[str, Any]) -> None:
        """Send action to XLerobot (arms, base, head).
        
        Maps teleoperator actions from left_*/right_* format to left_arm_*/right_arm_* format
        before sending to XLerobot. This allows bi_so100_leader teleoperator actions to work
        with XLerobot's action format.
        
        Args:
            action: Dictionary of action values (may have left_*/right_* format from teleoperator)
        """
        # Map teleoperator actions from left_*/right_* format to left_arm_*/right_arm_* format
        mapped_action = {}
        for key, value in action.items():
            if key.startswith("left_") and not key.startswith("left_arm_"):
                # Map left_shoulder_pan.pos -> left_arm_shoulder_pan.pos
                mapped_key = key.replace("left_", "left_arm_", 1)
                mapped_action[mapped_key] = value
            elif key.startswith("right_") and not key.startswith("right_arm_"):
                # Map right_shoulder_pan.pos -> right_arm_shoulder_pan.pos
                mapped_key = key.replace("right_", "right_arm_", 1)
                mapped_action[mapped_key] = value
            else:
                # Pass through other actions (head, base, etc.) unchanged
                mapped_action[key] = value
        
        # Send mapped action to XLerobot
        self.xlerobot.send_action(mapped_action)

