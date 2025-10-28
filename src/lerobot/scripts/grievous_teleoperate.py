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
Simple script to control a robot from teleoperation with ZMQ observation broadcasting.

Example:

```shell
grievous-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/tty.usbmodem58760431541 \
    --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 1920, height: 1080, fps: 30}}" \
    --robot.id=black \
    --teleop.type=so101_leader \
    --teleop.port=/dev/tty.usbmodem58760431551 \
    --teleop.id=blue \
    --remote_client_address=tcp://*:5555
```

Example teleoperation with bimanual so100:

```shell
grievous-teleoperate \
  --robot.type=bi_so100_follower \
  --robot.left_arm_port=/dev/tty.usbmodem5A460851411 \
  --robot.right_arm_port=/dev/tty.usbmodem5A460812391 \
  --robot.id=bimanual_follower \
  --robot.cameras='{
    left: {"type": "opencv", "index_or_path": 0, "width": 1920, "height": 1080, "fps": 30},
    top: {"type": "opencv", "index_or_path": 1, "width": 1920, "height": 1080, "fps": 30},
    right: {"type": "opencv", "index_or_path": 2, "width": 1920, "height": 1080, "fps": 30}
  }' \
  --teleop.type=bi_so100_leader \
  --teleop.left_arm_port=/dev/tty.usbmodem5A460828611 \
  --teleop.right_arm_port=/dev/tty.usbmodem5A460826981 \
  --teleop.id=bimanual_leader \
  --remote_client_address=tcp://*:5555
```

"""

import logging
import pickle
import time
from dataclasses import asdict, dataclass
from pprint import pformat
from typing import Any

import zmq

from lerobot.cameras.configs import ColorMode, Cv2Rotation, CameraConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig  # noqa: F401
from lerobot.configs import parser
from lerobot.processor import (
    RobotAction,
    RobotObservation,
    RobotProcessorPipeline,
    make_default_processors,
)
from lerobot.robots import (  # noqa: F401
    Robot,
    RobotConfig,
    bi_so100_follower,
    hope_jr,
    koch_follower,
    make_robot_from_config,
    so100_follower,
    so101_follower,
)
from lerobot.teleoperators import (  # noqa: F401
    Teleoperator,
    TeleoperatorConfig,
    bi_so100_leader,
    gamepad,
    homunculus,
    koch_leader,
    make_teleoperator_from_config,
    so100_leader,
    so101_leader,
)
from lerobot.utils.import_utils import register_third_party_devices
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import init_logging, move_cursor_up
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data


@dataclass
class GrievousTeleoperateConfig:
    # TODO: pepijn, steven: if more robots require multiple teleoperators (like lekiwi) its good to make this possibele in teleop.py and record.py with List[Teleoperator]
    teleop: TeleoperatorConfig
    robot: RobotConfig
    # Limit the maximum frames per second.
    fps: int = 60
    teleop_time_s: float | None = None
    # Remote client address for sending observations via ZMQ (e.g., "tcp://*:5555" or "tcp://0.0.0.0:5555")
    # Remote clients should connect to this address using a PULL socket to receive observations
    remote_client_address: str | None = None


def teleop_loop(
    teleop: Teleoperator,
    robot: Robot,
    fps: int,
    teleop_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_observation_processor: RobotProcessorPipeline[RobotObservation, RobotObservation],
    duration: float | None = None,
    zmq_socket: Any | None = None,
):
    """
    This function continuously reads actions from a teleoperation device, processes them through optional
    pipelines, sends them to a robot, optionally sends observations to a remote client via ZMQ,
    and optionally displays the robot's state. The loop runs at a specified frequency until a set
    duration is reached or it is manually interrupted.

    Args:
        teleop: The teleoperator device instance providing control actions.
        robot: The robot instance being controlled.
        fps: The target frequency for the control loop in frames per second.
        duration: The maximum duration of the teleoperation loop in seconds. If None, the loop runs indefinitely.
        teleop_action_processor: An optional pipeline to process raw actions from the teleoperator.
        robot_action_processor: An optional pipeline to process actions before they are sent to the robot.
        robot_observation_processor: An optional pipeline to process raw observations from the robot.
        zmq_socket: Optional ZMQ socket for sending observations to a remote client.
    """

    display_len = max(len(key) for key in robot.action_features)
    start = time.perf_counter()

    while True:
        loop_start = time.perf_counter()

        # Get robot observation
        # Not really needed for now other than for visualization
        # teleop_action_processor can take None as an observation
        # given that it is the identity processor as default
        obs = robot.get_observation()

        # Get teleop action
        raw_action = teleop.get_action()

        # Process teleop action through pipeline
        teleop_action = teleop_action_processor((raw_action, obs))

        # Process action for robot through pipeline
        robot_action_to_send = robot_action_processor((teleop_action, obs))

        # Send processed action to robot (robot_action_processor.to_output should return dict[str, Any])
        _ = robot.send_action(robot_action_to_send)

        # Send observation to remote client via ZMQ if socket is provided
        if zmq_socket is not None:
            try:
                # Get camera data from robot
                camera_data = {}
                if hasattr(robot, 'cameras') and robot.cameras:
                    for cam_name, camera in robot.cameras.items():
                        if camera.is_connected:
                            try:
                                # Get the latest frame from camera
                                frame = camera.get_frame()
                                if frame is not None:
                                    camera_data[cam_name] = frame
                            except Exception as e:
                                logging.debug(f"Failed to get frame from camera {cam_name}: {e}")
                
                # Serialize observation data including camera frames
                obs_data = {
                    "observation": obs,
                    "action": robot_action_to_send,
                    "camera_data": camera_data,
                    "timestamp": time.time(),
                }
                serialized = pickle.dumps(obs_data)
                # Use non-blocking send to avoid stalling if no receiver is present
                zmq_socket.send(serialized, zmq.NOBLOCK)
            except zmq.Again:
                # No receiver is available, continue without error
                pass
            except Exception as e:
                logging.warning(f"Failed to send observation via ZMQ: {e}")

        dt_s = time.perf_counter() - loop_start
        busy_wait(1 / fps - dt_s)
        loop_s = time.perf_counter() - loop_start

        if duration is not None and time.perf_counter() - start >= duration:
            return


@parser.wrap()
def grievous_teleoperate(cfg: GrievousTeleoperateConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))

    teleop = make_teleoperator_from_config(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    # Set up ZMQ socket for sending observations if remote address is provided
    zmq_context = None
    zmq_socket = None
    if cfg.remote_client_address:
        zmq_context = zmq.Context()
        zmq_socket = zmq_context.socket(zmq.PUSH)
        zmq_socket.setsockopt(zmq.CONFLATE, 1)  # Keep only the latest message if client is slow
        # Bind to local address where remote clients can connect
        zmq_socket.bind(cfg.remote_client_address)
        logging.info(f"ZMQ observation publisher bound to {cfg.remote_client_address}")
        logging.info("Remote clients should connect with PULL socket to this address to receive observations")

    teleop.connect()
    robot.connect()

    try:
        teleop_loop(
            teleop=teleop,
            robot=robot,
            fps=cfg.fps,
            duration=cfg.teleop_time_s,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
            zmq_socket=zmq_socket,
        )
    except KeyboardInterrupt:
        pass
    finally:
        if zmq_socket:
            zmq_socket.close()
        if zmq_context:
            zmq_context.term()
        teleop.disconnect()
        robot.disconnect()


def main():
    register_third_party_devices()
    grievous_teleoperate()


if __name__ == "__main__":
    main()

