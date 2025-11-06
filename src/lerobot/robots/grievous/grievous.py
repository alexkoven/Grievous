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

"""Grievous: Composite robot combining XLerobot (follower + base + head) with BiSO100Leader (leader arms).

Architecture:
- XLerobot provides: follower arms (2x SO-101), mobile base, head, cameras
- BiSO100Leader provides: leader arms (2x SO-101) for teleoperation input
- Observations include both follower state and leader state (prefixed with _leader)
- Actions are sent only to the follower (leader is input-only)
"""

import logging
from functools import cached_property
from typing import Any

from lerobot.robots.xlerobot import XLerobot
from lerobot.robots.xlerobot.config_xlerobot import XLerobotConfig
from lerobot.teleoperators.bi_so100_leader import BiSO100Leader
from lerobot.teleoperators.bi_so100_leader.config_bi_so100_leader import BiSO100LeaderConfig

from ..robot import Robot
from .config_grievous import GrievousConfig

logger = logging.getLogger(__name__)


class Grievous(Robot):
    """Grievous robot: XLerobot enhanced with leader arms for bimanual teleoperation.
    
    This composite robot combines:
    - XLerobot (2 follower arms, mobile base, head, 3 cameras)
    - BiSO100Leader (2 leader arms for teleoperation input)
    
    The leader arms provide teleoperation input (read-only in observations),
    while the follower arms, base, and head are controlled via actions.
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

        # Create BiSO100Leader configuration (leader arms for teleoperation)
        leader_config = BiSO100LeaderConfig(
            id=f"{config.id}_leader" if config.id else None,
            calibration_dir=config.calibration_dir,
            left_arm_port=config.leader_left_arm_port,
            right_arm_port=config.leader_right_arm_port,
        )

        # Instantiate composed components
        self.xlerobot = XLerobot(xlerobot_config)
        self.leader_arms = BiSO100Leader(leader_config)

        logger.info(f"Grievous robot initialized: {config.id}")

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        """Observation features include follower state + leader state.
        
        Follower features come from XLerobot (arms, base, head, cameras).
        Leader features are suffixed with '_leader' to distinguish from follower.
        
        Returns:
            Dictionary mapping feature names to types/shapes
        """
        # Get follower features (XLerobot: arms, base, head, cameras)
        follower_features = self.xlerobot.observation_features.copy()
        
        # Get leader features and add '_leader' suffix
        leader_features = {
            f"{key}_leader": value
            for key, value in self.leader_arms.action_features.items()
        }
        
        # Combine both
        return {**follower_features, **leader_features}

    @cached_property
    def action_features(self) -> dict[str, type]:
        """Action features include only follower controls (XLerobot).
        
        Leader arms are input-only (for teleoperation), so they don't appear in actions.
        
        Returns:
            Dictionary mapping action names to types
        """
        # Actions control only the follower (XLerobot)
        return self.xlerobot.action_features

    @property
    def is_connected(self) -> bool:
        """Check if both XLerobot and leader arms are connected."""
        return self.xlerobot.is_connected and self.leader_arms.is_connected

    @property
    def is_calibrated(self) -> bool:
        """Check if both XLerobot and leader arms are calibrated."""
        return self.xlerobot.is_calibrated and self.leader_arms.is_calibrated

    def connect(self, calibrate: bool = True) -> None:
        """Connect both XLerobot and leader arms.
        
        Args:
            calibrate: Whether to calibrate after connecting
        """
        logger.info("Connecting Grievous components...")
        self.xlerobot.connect(calibrate=calibrate)
        self.leader_arms.connect(calibrate=calibrate)
        logger.info("Grievous connected successfully")

    def disconnect(self) -> None:
        """Disconnect both XLerobot and leader arms."""
        logger.info("Disconnecting Grievous components...")
        self.xlerobot.disconnect()
        self.leader_arms.disconnect()
        logger.info("Grievous disconnected")

    def calibrate(self) -> None:
        """Calibrate both XLerobot and leader arms."""
        logger.info("Calibrating Grievous components...")
        self.xlerobot.calibrate()
        self.leader_arms.calibrate()
        logger.info("Grievous calibrated successfully")

    def configure(self) -> None:
        """Configure both XLerobot and leader arms.
        
        Sets motor parameters, control modes, and initial state.
        """
        logger.info("Configuring Grievous components...")
        self.xlerobot.configure()
        self.leader_arms.configure()
        logger.info("Grievous configured successfully")

    def get_observation(self) -> dict[str, Any]:
        """Get combined observation from follower and leader.
        
        Combines:
        - XLerobot observation (follower arms, base, head, cameras)
        - Leader arms positions (suffixed with '_leader')
        
        Returns:
            Dictionary with all observation data
        """
        # Get follower observation (XLerobot)
        observation = self.xlerobot.get_observation()
        
        # Get leader observation (BiSO100Leader.get_action() reads leader state)
        leader_observation = self.leader_arms.get_action()
        
        # Add leader data with '_leader' suffix
        for key, value in leader_observation.items():
            observation[f"{key}_leader"] = value
        
        return observation

    def send_action(self, action: dict[str, Any]) -> None:
        """Send action to follower (XLerobot only).
        
        Leader arms are input-only, so actions are not sent to them.
        
        Args:
            action: Dictionary of action values for follower
        """
        # Send action only to XLerobot (follower)
        self.xlerobot.send_action(action)

