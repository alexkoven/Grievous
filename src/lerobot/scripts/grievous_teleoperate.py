# COPIED: Copyright header from lerobot_teleoperate.py
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

from json import JSONDecodeError
import json
import logging
import pickle
import time
from dataclasses import asdict, dataclass
from pprint import pformat
from typing import Any

from lerobot import cameras
from lerobot.cameras import camera
import zmq
import cv2
import base64

import threading

from lerobot.cameras.configs import ColorMode, Cv2Rotation, CameraConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig  # noqa: F401
# COPIED: Core imports from lerobot_teleoperate.py
from lerobot.configs import parser
from lerobot.processor import (
    RobotAction,
    RobotObservation,
    RobotProcessorPipeline,
    make_default_processors,
)
# COPIED: Robot imports from lerobot_teleoperate.py (lines 70-78)
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
# COPIED: Teleoperator imports from lerobot_teleoperate.py (lines 80-89)
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
# COPIED: Utility imports from lerobot_teleoperate.py (lines 91-94)
from lerobot.utils.import_utils import register_third_party_devices
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import init_logging, move_cursor_up
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data


# MODIFIED: Config dataclass based on TeleoperateConfig from lerobot_teleoperate.py (lines 97-106)
# Removed: display_data field
# Added: remote_client_address field
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

# NEW: Custom threaded camera reader class for encoding and buffering camera frames
# Note: Duplicates functionality from camera's built-in async_read() and _read_loop()
class CameraReader:
    def __init__(self, cam, name=None):
        self.cam = cam
        self.name = name or "camera"
        self.latest_frame = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        print(f"[{self.name}] camera thread started with {self.cam} camera")

    def _loop(self):
        # NEW: Camera reading loop with encoding (inspired by xlerobot_host.py/lekiwi_host.py image encoding)
        frame_count = 0
        last_log = time.time()
        while self.running:
            if self.cam.is_connected:
                try:
                    frame = None;
                    frame = self.cam.async_read()
                    if frame is not None:
                        frame = cv2.resize(frame, (512,512), interpolation=cv2.INTER_AREA)
                        # COPIED: Image encoding pattern from xlerobot_host.py (lines 93-101) and lekiwi_host.py (lines 102-110)
                        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                        result, encframe = cv2.imencode('.jpg', frame, encode_param)
                        if result:
                            with self.lock:
                                self.latest_frame = base64.b64encode(encframe).decode("utf-8")
                except Exception as e:
                    logging.warning(f"[{self.name}] camera error: {e}")
                time.sleep(0.005)

    def get_latest_frame(self):
        with self.lock:
            return self.latest_frame

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            logging.info(f"[{self.name}] camera thread stopped")

# MODIFIED: teleop_loop function based on lerobot_teleoperate.py (lines 109-181)
# Removed: display_data parameter and visualization code
# Added: zmq_socket and camera_readers parameters for observation broadcasting
def teleop_loop(
    teleop: Teleoperator,
    robot: Robot,
    fps: int,
    teleop_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_observation_processor: RobotProcessorPipeline[RobotObservation, RobotObservation],
    duration: float | None = None,
    zmq_socket: Any | None = None,
    camera_readers: Any | None = None,
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
    # COPIED: Start time tracking from lerobot_teleoperate.py (line 136)
    start = time.perf_counter()

    # COPIED: Main loop structure from lerobot_teleoperate.py (lines 138-181)
    while True:
        loop_start = time.perf_counter()

        # COPIED: Get robot observation from lerobot_teleoperate.py (lines 141-145)
        # Get robot observation
        # Not really needed for now other than for visualization
        # teleop_action_processor can take None as an observation
        # given that it is the identity processor as default
        obs = robot.get_observation()

        # COPIED: Get and process teleop action from lerobot_teleoperate.py (lines 147-154)
        # Get teleop action
        raw_action = teleop.get_action()

        # Process teleop action through pipeline
        teleop_action = teleop_action_processor((raw_action, obs))

        # Process action for robot through pipeline
        robot_action_to_send = robot_action_processor((teleop_action, obs))

        # COPIED: Send action to robot from lerobot_teleoperate.py (lines 156-157)
        # Send processed action to robot (robot_action_processor.to_output should return dict[str, Any])
        _ = robot.send_action(robot_action_to_send)

        # NEW: ZMQ observation broadcasting block (replaces display_data visualization from lerobot_teleoperate.py lines 159-173)
        # Similar pattern to xlerobot_host.py and lekiwi_host.py but integrated into teleoperation loop
        # Send observation to remote client via ZMQ if socket is provided
        if zmq_socket is not None:
            try:
                # Get camera data from robot
                camera_data = {}
                if hasattr(robot, 'cameras') and robot.cameras:
                    for cam_name, reader in camera_readers.items():
                        frame = reader.get_latest_frame()
                        if frame is not None:
                            # frame = cv2.resize(frame, (512,512), interpolation=cv2.INTER_AREA)
                            # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                            # result, encframe = cv2.imencode('.jpg', frame, encode_param)
                            # if result:
                            #     camera_data[cam_name] = base64.b64encode(encframe).decode("utf-8")
                            camera_data[cam_name] = frame

                full_obs = obs | camera_data
                
                # Serialize observation data including camera frames
                obs_data = {
                    "observation": full_obs,
                    "action": robot_action_to_send,
                    "timestamp": time.time(),
                }
                serialized = pickle.dumps(obs_data)
                # COPIED: Non-blocking send pattern from xlerobot_host.py (lines 104-107) and lekiwi_host.py (lines 113-116)
                # Use non-blocking send to avoid stalling if no receiver is present
                zmq_socket.send(serialized, zmq.NOBLOCK)
            except zmq.Again:
                # No receiver is available, continue without error
                pass
            except Exception as e:
                logging.warning(f"Failed to send observation via ZMQ: {e}")

        # COPIED: Timing and busy wait from lerobot_teleoperate.py (lines 175-178)
        dt_s = time.perf_counter() - loop_start
        busy_wait(1 / fps - dt_s)
        loop_s = time.perf_counter() - loop_start
        print(f"\ntime: {loop_s * 1e3:.2f}ms ({1 / loop_s:.0f} Hz)")

        # COPIED: Duration check from lerobot_teleoperate.py (lines 180-181)
        if duration is not None and time.perf_counter() - start >= duration:
            return


# MODIFIED: Main function based on teleoperate() from lerobot_teleoperate.py (lines 184-215)
# Removed: Rerun initialization and shutdown
# Added: ZMQ socket setup and camera reader thread management
@parser.wrap()
def grievous_teleoperate(cfg: GrievousTeleoperateConfig):
    # COPIED: Logging setup from lerobot_teleoperate.py (lines 186-187)
    init_logging()
    logging.info(pformat(asdict(cfg)))

    # COPIED: Create teleop and robot from lerobot_teleoperate.py (lines 191-193)
    teleop = make_teleoperator_from_config(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    # NEW: ZMQ socket setup (similar pattern to xlerobot_host.py lines 31-38 and lekiwi_host.py lines 41-48)
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

    # COPIED: Connect devices from lerobot_teleoperate.py (lines 195-196)
    teleop.connect()
    robot.connect()
    # NEW: Camera reader thread initialization
    camera_readers = {}
    if hasattr(robot, "cameras") and robot.cameras:
        for cam_name, cam in robot.cameras.items():
            reader = CameraReader(cam, cam_name)
            reader.start()
            camera_readers[cam_name] = reader


    # COPIED: Try/except/finally structure from lerobot_teleoperate.py (lines 198-215)
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
            camera_readers=camera_readers,
        )
    except KeyboardInterrupt:
        pass
    finally:
        # NEW: Camera reader cleanup
        for reader in camera_readers.values():
            reader.stop()
        # NEW: ZMQ socket cleanup
        if zmq_socket:
            zmq_socket.close()
        if zmq_context:
            zmq_context.term()
        # COPIED: Device disconnection from lerobot_teleoperate.py (lines 214-215)
        teleop.disconnect()
        robot.disconnect()


# COPIED: main() function from lerobot_teleoperate.py (lines 218-220)
def main():
    register_third_party_devices()
    grievous_teleoperate()


if __name__ == "__main__":
    main()

