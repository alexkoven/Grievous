#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

import json
import logging
from functools import cached_property

import zmq

from lerobot.teleoperators.so100_leader.config_so100_leader import SO100LeaderConfig
from lerobot.teleoperators.so100_leader.so100_leader import SO100Leader

from ..teleoperator import Teleoperator
from .config_grievous_leader import GrievousLeaderConfig

logger = logging.getLogger(__name__)


class GrievousLeader(Teleoperator):
    """
    [Bimanual SO-100 Leader Arms](https://github.com/TheRobotStudio/SO-ARM100) designed by TheRobotStudio
    This bimanual leader arm can also be easily adapted to use SO-101 leader arms, just replace the SO100Leader class with SO101Leader and SO100LeaderConfig with SO101LeaderConfig.
    """

    config_class = GrievousLeaderConfig
    name = "grievous_leader"

    def __init__(self, config: GrievousLeaderConfig):
        super().__init__(config)
        self.config = config

        # Network configuration
        self.remote_ip = config.remote_ip
        self.port_zmq_cmd = config.port_zmq_cmd

        # ZMQ sockets (initialized in connect())
        self.zmq_context = None
        self.zmq_cmd_socket = None

        # left_arm_config = SO100LeaderConfig(
        #     id=f"{config.id}_left" if config.id else None,
        #     calibration_dir=config.calibration_dir,
        #     port=config.left_arm_port,
        # )

        # right_arm_config = SO100LeaderConfig(
        #     id=f"{config.id}_right" if config.id else None,
        #     calibration_dir=config.calibration_dir,
        #     port=config.right_arm_port,
        # )

        # self.left_arm = SO100Leader(left_arm_config)
        # self.right_arm = SO100Leader(right_arm_config)

    @cached_property
    def action_features(self) -> dict[str, type]:
        """Action features matching grievous_client format (left_arm_*, right_arm_*)."""
        # Standard SO-100/101 motor names
        motor_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ]
        return {f"left_arm_{motor}.pos": float for motor in motor_names} | {
            f"right_arm_{motor}.pos": float for motor in motor_names
        }

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.zmq_cmd_socket is not None

    def connect(self, calibrate: bool = True) -> None:
        # self.left_arm.connect(calibrate)
        # self.right_arm.connect(calibrate)

        # Create ZMQ context and command socket
        self.zmq_context = zmq.Context()
        self.zmq_cmd_socket = self.zmq_context.socket(zmq.PULL)
        zmq_cmd_locator = f"tcp://{self.remote_ip}:{self.port_zmq_cmd}"
        self.zmq_cmd_socket.connect(zmq_cmd_locator)
        self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)  # Keep only latest message
        logger.info(f"Command socket connected to {zmq_cmd_locator}")

    @property
    def is_calibrated(self) -> bool:
        return True  # Calibration handled on host side

    def calibrate(self) -> None:
        pass  # Calibration handled on host side

    def configure(self) -> None:
        pass  # Configuration handled on host side

    def setup_motors(self) -> None:
        pass  # Motor setup handled on host side

    def get_action(self) -> dict[str, float]:
        action = {}

        # Receive action via ZMQ if connected
        if self.zmq_cmd_socket:
            try:
                # Non-blocking receive with NOBLOCK flag - returns immediately if no data available
                action_dict = json.loads(self.zmq_cmd_socket.recv_string(zmq.NOBLOCK))
                # Map leader arm format (left_*, right_*) to follower arm format (left_arm_*, right_arm_*)
                # to match what grievous_client expects
                for key, value in action_dict.items():
                    if key.startswith("left_") and not key.startswith("left_arm_"):
                        # Map left_shoulder_pan.pos -> left_arm_shoulder_pan.pos
                        mapped_key = key.replace("left_", "left_arm_", 1)
                        action[mapped_key] = value
                    elif key.startswith("right_") and not key.startswith("right_arm_"):
                        # Map right_shoulder_pan.pos -> right_arm_shoulder_pan.pos
                        mapped_key = key.replace("right_", "right_arm_", 1)
                        action[mapped_key] = value
                    else:
                        # Pass through other actions (head, base, etc.) unchanged
                        action[key] = value
            except zmq.Again:
                # No data available - this is normal, just return default actions
                logger.debug("No action available from ZMQ (non-blocking)")
            except Exception as e:
                logger.error(f"Error receiving action via ZMQ: {e}")

        # Add default values for head and base motors if not present
        # These are required by grievous_client action_features but may not be in ZMQ message
        if "head_motor_1.pos" not in action:
            action["head_motor_1.pos"] = 0.0
        if "head_motor_2.pos" not in action:
            action["head_motor_2.pos"] = 0.0
        if "x.vel" not in action:
            action["x.vel"] = 0.0
        if "y.vel" not in action:
            action["y.vel"] = 0.0
        if "theta.vel" not in action:
            action["theta.vel"] = 0.0

        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # Feedback is handled on host side via ZMQ
        pass

    def disconnect(self) -> None:
        # Close ZMQ sockets
        if self.zmq_cmd_socket:
            self.zmq_cmd_socket.close()
        if self.zmq_context:
            self.zmq_context.term()
        self.zmq_cmd_socket = None
        self.zmq_context = None
        logger.info("ZMQ command socket disconnected")

