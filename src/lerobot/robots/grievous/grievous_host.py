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

"""Grievous Host Daemon - Runs on RPi5 to manage physical robot hardware.

This daemon:
- Instantiates the Grievous robot (follower arms + base + head + leader arms)
- Opens ZMQ sockets to receive commands and send observations
- Runs a control loop that:
  * Receives action commands from remote client (laptop)
  * Sends actions to follower (XLerobot)
  * Reads observations from both follower and leader
  * Encodes camera images to base64
  * Sends observations back to client
  * Implements watchdog timer for safety (stops base if no commands)
"""

import base64
import json
import logging
import time

import cv2
import numpy as np
import zmq

from lerobot.processor import (
    RobotAction,
    RobotObservation,
    RobotProcessorPipeline,
    make_default_processors,
)

from .grievous import Grievous
from .config_grievous import GrievousConfig, GrievousHostConfig

logger = logging.getLogger(__name__)


class GrievousHost:
    """ZMQ-based host daemon for Grievous robot running on RPi5.
    
    Manages bidirectional communication with remote client:
    - Receives actions via ZMQ PULL socket
    - Sends observations via ZMQ PUSH socket
    """

    def __init__(self, config: GrievousHostConfig):
        """Initialize ZMQ sockets for command and observation streaming.
        
        Args:
            config: Host configuration (ports, timeouts, loop frequency)
        """
        self.zmq_context = zmq.Context()
        
        # Command socket: receive actions from client
        self.zmq_cmd_socket = self.zmq_context.socket(zmq.PULL)
        self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)  # Keep only latest message
        self.zmq_cmd_socket.bind(f"tcp://*:{config.port_zmq_cmd}")
        logger.info(f"Command socket bound to tcp://*:{config.port_zmq_cmd}")
        
        # Observation socket: send observations to client
        self.zmq_observation_socket = self.zmq_context.socket(zmq.PUSH)
        self.zmq_observation_socket.setsockopt(zmq.CONFLATE, 1)  # Keep only latest message
        self.zmq_observation_socket.bind(f"tcp://*:{config.port_zmq_observations}")
        logger.info(f"Observation socket bound to tcp://*:{config.port_zmq_observations}")
        
        # Configuration
        self.connection_time_s = config.connection_time_s
        self.watchdog_timeout_ms = config.watchdog_timeout_ms
        self.max_loop_freq_hz = config.max_loop_freq_hz
        
        logger.info(f"GrievousHost initialized: watchdog={config.watchdog_timeout_ms}ms, freq={config.max_loop_freq_hz}Hz")

    def disconnect(self) -> None:
        """Close ZMQ sockets and terminate context."""
        logger.info("Closing GrievousHost ZMQ sockets...")
        self.zmq_observation_socket.close()
        self.zmq_cmd_socket.close()
        self.zmq_context.term()
        logger.info("GrievousHost disconnected")


def main():
    """Main entry point for Grievous host daemon.
    
    Runs control loop that:
    1. Receives actions from remote client
    2. Sends actions to Grievous follower
    3. Reads observations from Grievous (follower + leader)
    4. Encodes camera images to base64
    5. Sends observations to remote client
    6. Implements watchdog safety timer
    """
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    
    logger.info("Configuring Grievous robot...")
    # Use proper ID for calibration management (avoids None collisions)
    robot_config = GrievousConfig(id="grievous_robot")
    robot = Grievous(robot_config)
    
    logger.info("Connecting Grievous robot (using existing calibration)...")
    robot.connect(calibrate=False)  # Use existing calibration from cache
    logger.info("Grievous connected successfully")
    
    logger.info("Starting GrievousHost daemon...")
    host_config = GrievousHostConfig()
    host = GrievousHost(host_config)

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()
    
    last_cmd_time = time.time()
    watchdog_active = False
    logger.info("Waiting for commands from remote client...")
    
    try:
        # Main control loop
        start = time.perf_counter()
        duration = 0
        
        while duration < host.connection_time_s:
            loop_start_time = time.time()
            
            # Adapt when base and head controls are implemented
            # # 1. Receive action commands from client
            # try:
            #     msg = host.zmq_cmd_socket.recv_string(zmq.NOBLOCK)
            #     data = dict(json.loads(msg))
                
            #     # Send action to follower (XLerobot component)
            #     robot.send_action(data)
            #     print(f"Sent action to follower: {data}")
                
            #     # Reset watchdog timer
            #     last_cmd_time = time.time()
            #     watchdog_active = False
                
            # except zmq.Again:
            #     # No command available (non-blocking)
            #     if not watchdog_active:
            #         logger.debug("No command available")
            # except Exception as e:
            #     logger.debug(f"Message fetching failed: {e}")
            
            # 2. Check watchdog timer
            now = time.time()
            if (now - last_cmd_time > host.watchdog_timeout_ms / 1000) and not watchdog_active:
                logger.warning(
                    f"Command not received for {host.watchdog_timeout_ms}ms. Stopping base for safety."
                )
                watchdog_active = True
                # Stop the mobile base (safety feature)
                robot.xlerobot.stop_base()
            
            # 3. Get observation and actionfrom Grievous (follower + cameras)
            last_observation = robot.get_observation()
            action = robot.get_action()
            # Action processors should be better understood, and potentially removed
            teleop_action = teleop_action_processor((action, last_observation))
            robot_action = robot_action_processor((teleop_action, last_observation))
            robot.send_action(action)
            
            # 4. Encode camera images to base64 for network transmission
            for cam_key in robot.xlerobot.cameras.keys():
                if cam_key in last_observation:
                    # Check if image is valid (not None and not empty)
                    try:
                        img = last_observation[cam_key]
                        if img is None or not isinstance(img, np.ndarray) or img.size == 0:
                            logger.debug(f"Camera {cam_key} returned empty/invalid image, skipping encode")
                            last_observation[cam_key] = ""
                            continue
                        
                        ret, buffer = cv2.imencode(
                            ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                        )
                        if ret:
                            last_observation[cam_key] = base64.b64encode(buffer).decode("utf-8")
                        else:
                            logger.warning(f"Failed to encode camera {cam_key}")
                            last_observation[cam_key] = ""
                    except Exception as e:
                        logger.error(f"Failed to encode camera {cam_key}: {e}")
                        last_observation[cam_key] = ""
                    
            
            # 5. Send observation to remote client
            try:
                host.zmq_observation_socket.send_string(
                    json.dumps(last_observation), flags=zmq.NOBLOCK
                )
            except zmq.Again:
                logger.debug("Dropping observation, no client connected")
            except Exception as e:
                logger.error(f"Failed to send observation: {e}")
            
            # 6. Rate limiting
            elapsed = time.time() - loop_start_time
            sleep_time = max(1 / host.max_loop_freq_hz - elapsed, 0)
            time.sleep(sleep_time)
            
            duration = time.perf_counter() - start
        
        logger.info(f"Connection time limit reached ({host.connection_time_s}s). Shutting down.")
    
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received. Shutting down...")
    
    finally:
        logger.info("Cleaning up Grievous host...")
        robot.disconnect()
        host.disconnect()
        logger.info("Grievous host shutdown complete")


if __name__ == "__main__":
    main()

