# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

"""
Simple remote viewer for grievous_teleoperate observations via ZMQ.

Usage:

```shell
grievous-viewer --remote_server_address=tcp://192.168.1.100:5555
```

This script connects to a running grievous_teleoperate instance and displays
the robot observations in Rerun.

"""

import logging
import pickle
import time
from dataclasses import dataclass

import zmq

import rerun as rr

from lerobot.configs import parser
from lerobot.processor import (
    RobotObservation,
    RobotProcessorPipeline,
    make_default_processors,
)
from lerobot.utils.utils import init_logging
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data


@dataclass
class GrievousViewerConfig:
    # Remote server address to connect to (e.g., "tcp://192.168.1.100:5555")
    remote_server_address: str
    # Name for the rerun session
    session_name: str = "grievous_remote_viewer"


@parser.wrap()
def grievous_viewer(cfg: GrievousViewerConfig):
    init_logging()
    logging.info(f"Connecting to {cfg.remote_server_address}")
    
    # Initialize Rerun
    init_rerun(session_name=cfg.session_name)
    
    # Set up ZMQ connection
    zmq_context = zmq.Context()
    zmq_socket = zmq_context.socket(zmq.PULL)
    zmq_socket.connect(cfg.remote_server_address)
    
    logging.info("Connected! Receiving observations... Press Ctrl+C to stop.")
    
    try:
        while True:
            # Receive observation data
            data = zmq_socket.recv()
            obs_data = pickle.loads(data)
            
            # Extract observation and action
            obs = obs_data["observation"]
            action = obs_data["action"]
            
            # Get processors (using defaults from the teleoperate side)
            _, _, robot_observation_processor = make_default_processors()
            
            # Process observation
            obs_transition = robot_observation_processor(obs)
            
            # Log to Rerun
            log_rerun_data(
                observation=obs_transition,
                action=action,
            )
            
            # Print timestamp
            timestamp = obs_data.get("timestamp", time.time())
            print(f"\rReceived observation at {time.ctime(timestamp)}", end="")
            
    except KeyboardInterrupt:
        logging.info("\nStopping viewer...")
    except Exception as e:
        logging.error(f"Error receiving data: {e}")
    finally:
        zmq_socket.close()
        zmq_context.term()
        rr.rerun_shutdown()


def main():
    grievous_viewer()


if __name__ == "__main__":
    main()

