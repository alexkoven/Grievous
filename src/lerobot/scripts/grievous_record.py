# COPIED: Copyright header from lerobot_record.py
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
Records a dataset from teleoperation with ZMQ observation broadcasting.

Example:

```shell
grievous-record \
    --robot.type=so101_follower \
    --robot.port=/dev/tty.usbmodem58760431541 \
    --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 1920, height: 1080, fps: 30}}" \
    --robot.id=black \
    --teleop.type=so101_leader \
    --teleop.port=/dev/tty.usbmodem58760431551 \
    --teleop.id=blue \
    --dataset.repo_id=<my_username>/<my_dataset_name> \
    --dataset.num_episodes=2 \
    --dataset.single_task="Grab the cube" \
    --remote_client_address=tcp://*:5555
```

Example recording with bimanual so100:

```shell
grievous-record \
  --robot.type=bi_so100_follower \
  --robot.left_arm_port=/dev/tty.usbmodem5A460851411 \
  --robot.right_arm_port=/dev/tty.usbmodem5A460812391 \
  --robot.id=bimanual_follower \
  --robot.cameras='{
    left: {"type": "opencv", "index_or_path": 0, "width": 1920, "height": 1080, fps: 30},
    top: {"type": "opencv", "index_or_path": 1, "width": 1920, "height": 1080, fps: 30},
    right: {"type": "opencv", "index_or_path": 2, "width": 1920, "height": 1080, fps: 30}
  }' \
  --teleop.type=bi_so100_leader \
  --teleop.left_arm_port=/dev/tty.usbmodem5A460828611 \
  --teleop.right_arm_port=/dev/tty.usbmodem5A460826981 \
  --teleop.id=bimanual_leader \
  --dataset.repo_id=${HF_USER}/bimanual-so100-handover-cube \
  --dataset.num_episodes=25 \
  --dataset.single_task="Grab and handover the red cube to the other arm" \
  --remote_client_address=tcp://*:5555
```
"""

import json
import logging
import pickle
import shutil
import tempfile
import time
import threading
from contextlib import nullcontext
from dataclasses import asdict, dataclass, field
from pathlib import Path
from pprint import pformat
from typing import Any

import cv2
import base64
import zmq

from lerobot.cameras import (  # noqa: F401
    CameraConfig,  # noqa: F401
)
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig  # noqa: F401
from lerobot.configs import parser
from lerobot.datasets.image_writer import safe_stop_image_writer
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.pipeline_features import aggregate_pipeline_dataset_features, create_initial_features
from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts, INFO_PATH
from lerobot.datasets.video_utils import VideoEncodingManager
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
    homunculus,
    koch_leader,
    make_teleoperator_from_config,
    so100_leader,
    so101_leader,
)
from lerobot.utils.constants import ACTION, HF_LEROBOT_HOME, OBS_STR
from lerobot.utils.control_utils import (
    init_keyboard_listener,
    is_headless,
    sanity_check_dataset_name,
    sanity_check_dataset_robot_compatibility,
)
from lerobot.utils.import_utils import register_third_party_devices
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import (
    init_logging,
    log_say,
)
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data


def safe_log_say(text: str, play_sounds: bool = True, blocking: bool = False):
    """
    Wrapper around log_say that gracefully handles missing text-to-speech commands.
    
    Args:
        text: Text to log and optionally speak
        play_sounds: Whether to attempt text-to-speech
        blocking: Whether to block until speech completes
    """
    try:
        log_say(text, play_sounds, blocking)
    except FileNotFoundError:
        # Text-to-speech command not available (e.g., spd-say), just log
        logging.info(text)
    except Exception as e:
        # Other errors with text-to-speech, log warning but continue
        logging.warning(f"Failed to use text-to-speech: {e}. Logging text instead.")
        logging.info(text)


@dataclass
class DatasetRecordConfig:
    # Dataset identifier. By convention it should match '{hf_username}/{dataset_name}' (e.g. `lerobot/test`).
    repo_id: str
    # A short but accurate description of the task performed during the recording (e.g. "Pick the Lego block and drop it in the box on the right.")
    single_task: str
    # Root directory where the dataset will be stored (e.g. 'dataset/path').
    root: str | Path | None = None
    # Limit the frames per second.
    fps: int = 30
    # Number of seconds for data recording for each episode.
    episode_time_s: int | float = 10
    # Number of seconds for resetting the environment after each episode.
    reset_time_s: int | float = 60
    # Number of episodes to record.
    num_episodes: int = 50
    # Encode frames in the dataset into video
    video: bool = True
    # Upload dataset to Hugging Face hub.
    push_to_hub: bool = True
    # Upload on private repository on the Hugging Face hub.
    private: bool = False
    # Add tags to your dataset on the hub.
    tags: list[str] | None = None
    # Number of subprocesses handling the saving of frames as PNG. Set to 0 to use threads only;
    # set to ≥1 to use subprocesses, each using threads to write images. The best number of processes
    # and threads depends on your system. We recommend 4 threads per camera with 0 processes.
    # If fps is unstable, adjust the thread count. If still unstable, try using 1 or more subprocesses.
    num_image_writer_processes: int = 0
    # Number of threads writing the frames as png images on disk, per camera.
    # Too many threads might cause unstable teleoperation fps due to main thread being blocked.
    # Not enough threads might cause low camera fps.
    num_image_writer_threads_per_camera: int = 4
    # Number of episodes to record before batch encoding videos
    # Set to 1 for immediate encoding (default behavior), or higher for batched encoding
    video_encoding_batch_size: int = 1
    # Rename map for the observation to override the image and state keys
    rename_map: dict[str, str] = field(default_factory=dict)

    def __post_init__(self):
        if self.single_task is None:
            raise ValueError("You need to provide a task as argument in `single_task`.")
 

@dataclass
class GrievousRecordConfig:
    robot: RobotConfig
    dataset: DatasetRecordConfig
    # Whether to control the robot with a teleoperator
    teleop: TeleoperatorConfig | None = None
    # Display all cameras on screen
    display_data: bool = False
    # Use vocal synthesis to read events.
    play_sounds: bool = False
    # Resume recording on an existing dataset.
    resume: bool = False
    # Remote client address for sending observations via ZMQ (e.g., "tcp://*:5555" or "tcp://0.0.0.0:5555")
    # Remote clients should connect to this address using a PULL socket to receive observations
    remote_client_address: str | None = None
    # Whether to save dataset locally. If False, only sends dataset frames over ZMQ for remote recording.
    # If True, saves locally and can push to Hugging Face Hub.
    save_locally: bool = False

    def __post_init__(self):
        if self.teleop is None:
            raise ValueError("Teleoperator is required for grievous-record")


# NEW: Custom threaded camera reader class for encoding and buffering camera frames
# COPIED from grievous_teleoperate.py with bug fix
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
        # Camera reading loop with encoding (inspired by xlerobot_host.py/lekiwi_host.py image encoding)
        frame_count = 0
        last_log = time.time()
        while self.running:
            if self.cam.is_connected:
                try:
                    frame = self.cam.read()
                    if frame is not None:
                        # # Image encoding pattern from xlerobot_host.py and lekiwi_host.py
                        # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                        # result, encframe = cv2.imencode('.jpg', frame, encode_param)
                        # if result:
                        #     with self.lock:
                        #         # Increases the filesize by 33%, but can be sent in JSON
                        #         self.latest_frame = base64.b64encode(encframe).decode("utf-8")
                        with self.lock:
                            self.latest_frame = frame
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


@safe_stop_image_writer
def record_loop(
    robot: Robot,
    events: dict,
    fps: int,
    teleop_action_processor: RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ],  # runs after teleop
    robot_action_processor: RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ],  # runs before robot
    robot_observation_processor: RobotProcessorPipeline[
        RobotObservation, RobotObservation
    ],  # runs after robot
    dataset: LeRobotDataset | None = None,
    teleop: Teleoperator | None = None,
    control_time_s: int | None = None,
    single_task: str | None = None,
    display_data: bool = False,
    zmq_socket: Any | None = None,
    camera_readers: dict[str, CameraReader] | None = None,
    save_locally: bool = True,
):
    """
    This function continuously reads actions from a teleoperation device, processes them through optional
    pipelines, sends them to a robot, records data to a dataset, and optionally sends observations
    to a remote client via ZMQ. The loop runs at a specified frequency until a set duration is reached
    or it is manually interrupted.

    Args:
        robot: The robot instance being controlled.
        events: Dictionary of keyboard events for controlling recording.
        fps: The target frequency for the control loop in frames per second.
        teleop_action_processor: An optional pipeline to process raw actions from the teleoperator.
        robot_action_processor: An optional pipeline to process actions before they are sent to the robot.
        robot_observation_processor: An optional pipeline to process raw observations from the robot.
        dataset: Optional LeRobotDataset to record frames to.
        teleop: The teleoperator device instance providing control actions.
        control_time_s: The maximum duration of the recording loop in seconds.
        single_task: Task description string for the current episode.
        display_data: Whether to display data using rerun.
        zmq_socket: Optional ZMQ socket for sending observations to a remote client.
        camera_readers: Optional dict of CameraReader instances for encoding camera frames.
    """
    if dataset is not None and dataset.fps != fps:
        raise ValueError(f"The dataset fps should be equal to requested fps ({dataset.fps} != {fps}).")

    timestamp = 0
    start_episode_t = time.perf_counter()
    while timestamp < control_time_s:
        start_loop_t = time.perf_counter()

        if events["exit_early"]:
            events["exit_early"] = False
            break

        # Get robot observation
        obs = robot.get_observation()

        # NEW: Explicitly read cameras from robot.cameras for dataset recording
        # This ensures camera data is included even if robot.get_observation() doesn't include it
        camera_data = {}
        if camera_readers:
            for cam_name, reader in camera_readers.items():
                frame = reader.get_latest_frame()
                if frame is not None:
                    camera_data[cam_name] = frame

        full_obs = obs | camera_data

        # Applies a pipeline to the raw robot observation, default is IdentityProcessor
        obs_processed = robot_observation_processor(full_obs)

        if dataset is not None:
            observation_frame = build_dataset_frame(dataset.features, obs_processed, prefix=OBS_STR)

        # Get teleop action
        if teleop is not None:
            act = teleop.get_action()

            # Applies a pipeline to the raw teleop action, default is IdentityProcessor
            act_processed_teleop = teleop_action_processor((act, obs))
        else:
            logging.info(
                "No teleoperator provided, skipping action generation."
                "This is likely to happen when resetting the environment without a teleop device."
                "The robot won't be at its rest position at the start of the next episode."
            )
            continue

        # Applies a pipeline to the action, default is IdentityProcessor
        action_values = act_processed_teleop
        robot_action_to_send = robot_action_processor((act_processed_teleop, obs))

        # Send action to robot
        _sent_action = robot.send_action(robot_action_to_send)

        # Build dataset frame (needed for both local saving and ZMQ transmission)
        dataset_frame = None
        if dataset is not None:
            action_frame = build_dataset_frame(dataset.features, action_values, prefix=ACTION)
            dataset_frame = {**observation_frame, **action_frame, "task": single_task}
            # Only save locally if save_locally is True
            if save_locally:
                dataset.add_frame(dataset_frame)

        # NEW: ZMQ observation broadcasting (similar to grievous_teleoperate.py)
        # Sends both real-time observations/actions and dataset frames for remote recording
        if zmq_socket is not None:
            try:
                # # Get camera data from camera readers
                # camera_data = {}
                # if camera_readers:
                #     for cam_name, reader in camera_readers.items():
                #         frame = reader.get_latest_frame()
                #         if frame is not None:
                #             camera_data[cam_name] = frame

                # full_obs = obs_processed | camera_data

                # Serialize observation data including camera frames
                obs_data = {
                    "observation": full_obs,
                    "action": robot_action_to_send,
                    "timestamp": time.time(),
                }
                
                # NEW: Include dataset frame if available (for remote dataset recording)
                if dataset_frame is not None:
                    # Convert numpy arrays to lists for JSON serialization (if needed)
                    # But using pickle, so numpy arrays are fine
                    obs_data["dataset_frame"] = dataset_frame
                    # Also include dataset features metadata for the remote client
                    if dataset is not None:
                        obs_data["dataset_features"] = dataset.features
                        obs_data["dataset_fps"] = dataset.fps
                
                # Use pickle for binary serialization (more efficient than JSON for numpy arrays)
                serialized = pickle.dumps(obs_data)
                # Non-blocking send to avoid stalling if no receiver is present
                zmq_socket.send(serialized)
            except zmq.Again:
                # No receiver is available, continue without error
                pass
            except Exception as e:
                logging.warning(f"Failed to send observation via ZMQ: {e}")

        if display_data and not is_headless():
            try:
                log_rerun_data(observation=obs_processed, action=action_values)
            except Exception as e:
                # Silently fail in headless environments or if rerun fails
                pass

        dt_s = time.perf_counter() - start_loop_t
        busy_wait(1 / fps - dt_s)

        timestamp = time.perf_counter() - start_episode_t


@parser.wrap()
def grievous_record(cfg: GrievousRecordConfig) -> LeRobotDataset:
    init_logging()
    logging.info(pformat(asdict(cfg)))
    
    # Only initialize Rerun if display_data is True and not in headless environment
    if cfg.display_data and not is_headless():
        try:
            init_rerun(session_name="recording")
        except Exception as e:
            logging.warning(f"Failed to initialize Rerun (may be headless): {e}. Continuing without visualization.")
            cfg.display_data = False
    elif cfg.display_data and is_headless():
        logging.warning("Display data requested but running in headless environment. Disabling visualization.")
        cfg.display_data = False

    robot = make_robot_from_config(cfg.robot)
    teleop = make_teleoperator_from_config(cfg.teleop) if cfg.teleop is not None else None

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    dataset_features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=create_initial_features(
                action=robot.action_features
            ),  # TODO(steven, pepijn): in future this should be come from teleop or policy
            use_videos=cfg.dataset.video,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=cfg.dataset.video,
        ),
    )

    # Dataset handling: only create if saving locally, otherwise use minimal structure for ZMQ streaming
    dataset = None
    if cfg.save_locally:
        # Check if dataset directory already exists and has a valid info.json file
        # If so, automatically switch to resume mode
        dataset_root = Path(cfg.dataset.root) if cfg.dataset.root else HF_LEROBOT_HOME / cfg.dataset.repo_id
        info_path = dataset_root / INFO_PATH
        if not cfg.resume and info_path.exists():
            logging.info(f"Dataset directory already exists at {dataset_root} with info.json. Automatically switching to resume mode.")
            cfg.resume = True

        if cfg.resume:
            dataset = LeRobotDataset(
                cfg.dataset.repo_id,
                root=cfg.dataset.root,
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
            )

            # Only start image writer if saving locally
            if hasattr(robot, "cameras") and len(robot.cameras) > 0:
                dataset.start_image_writer(
                    num_processes=cfg.dataset.num_image_writer_processes,
                    num_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
                )
            sanity_check_dataset_robot_compatibility(dataset, robot, cfg.dataset.fps, dataset_features)
        else:
            # Create empty dataset for local saving
            sanity_check_dataset_name(cfg.dataset.repo_id, None)
            dataset = LeRobotDataset.create(
                cfg.dataset.repo_id,
                cfg.dataset.fps,
                root=cfg.dataset.root,
                robot_type=robot.name,
                features=dataset_features,
                use_videos=cfg.dataset.video,
                image_writer_processes=cfg.dataset.num_image_writer_processes,
                image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
            )
        logging.info(f"Dataset created locally at: {dataset.root}")
    else:
        # Not saving locally - only streaming over ZMQ
        # Create a minimal dataset object in memory (using temp directory) just for building frames
        if cfg.remote_client_address:
            temp_root = Path(tempfile.mkdtemp(prefix="grievous_stream_"))
            logging.info(f"Streaming mode: Not saving locally, only streaming over ZMQ to {cfg.remote_client_address}")
            logging.info(f"Using temporary dataset structure at: {temp_root} (will be cleaned up)")
            try:
                dataset = LeRobotDataset.create(
                    cfg.dataset.repo_id,
                    cfg.dataset.fps,
                    root=temp_root,
                    robot_type=robot.name,
                    features=dataset_features,
                    use_videos=cfg.dataset.video,
                    image_writer_processes=0,  # No image writing when streaming
                    image_writer_threads=0,
                    batch_encoding_size=cfg.dataset.video_encoding_batch_size,
                )
                logging.info("Temporary dataset structure created for frame building (ZMQ streaming only)")
            except Exception as e:
                logging.warning(f"Failed to create temporary dataset structure: {e}. Will send raw observations over ZMQ.")
                dataset = None
        else:
            logging.warning("save_locally=False but no remote_client_address specified. No data will be saved or streamed!")

    # NEW: ZMQ socket setup (similar pattern to grievous_teleoperate.py)
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

    robot.connect()
    if teleop is not None:
        teleop.connect()

    # NEW: Camera reader thread initialization
    camera_readers = {}
    if hasattr(robot, "cameras") and robot.cameras:
        for cam_name, cam in robot.cameras.items():
            reader = CameraReader(cam, cam_name)
            reader.start()
            camera_readers[cam_name] = reader

    listener, events = init_keyboard_listener()

    # Only use VideoEncodingManager if saving locally
    video_manager = VideoEncodingManager(dataset) if cfg.save_locally else nullcontext()
    with video_manager:
        recorded_episodes = 0
        while recorded_episodes < cfg.dataset.num_episodes and not events["stop_recording"]:
            safe_log_say(f"Recording episode {dataset.num_episodes}", cfg.play_sounds)
            record_loop(
                robot=robot,
                events=events,
                fps=cfg.dataset.fps,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
                teleop=teleop,
                dataset=dataset,
                control_time_s=cfg.dataset.episode_time_s,
                single_task=cfg.dataset.single_task,
                display_data=cfg.display_data,
                zmq_socket=zmq_socket,
                camera_readers=camera_readers,
                save_locally=cfg.save_locally,
            )

            # Execute a few seconds without recording to give time to manually reset the environment
            # Skip reset for the last episode to be recorded
            if not events["stop_recording"] and (
                (recorded_episodes < cfg.dataset.num_episodes - 1) or events["rerecord_episode"]
            ):
                safe_log_say("Reset the environment", cfg.play_sounds)
                record_loop(
                    robot=robot,
                    events=events,
                    fps=cfg.dataset.fps,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    teleop=teleop,
                    dataset=dataset,
                    control_time_s=cfg.dataset.reset_time_s,
                    single_task=cfg.dataset.single_task,
                    display_data=cfg.display_data,
                    zmq_socket=zmq_socket,
                    camera_readers=camera_readers,
                    save_locally=cfg.save_locally,
                )

            if events["rerecord_episode"]:
                safe_log_say("Re-record episode", cfg.play_sounds)
                events["rerecord_episode"] = False
                events["exit_early"] = False
                if cfg.save_locally and dataset is not None:
                    dataset.clear_episode_buffer()
                continue

            # Only save episode locally if save_locally is True
            if cfg.save_locally and dataset is not None:
                dataset.save_episode()
            recorded_episodes += 1

    safe_log_say("Stop recording", cfg.play_sounds, blocking=True)

    # NEW: Camera reader cleanup
    for reader in camera_readers.values():
        reader.stop()

    # NEW: ZMQ socket cleanup
    if zmq_socket:
        zmq_socket.close()
    if zmq_context:
        zmq_context.term()

    robot.disconnect()
    if teleop is not None:
        teleop.disconnect()

    if not is_headless() and listener is not None:
        listener.stop()

    # Cleanup temporary dataset directory if streaming only
    if not cfg.save_locally and dataset is not None:
        temp_root = dataset.root
        if temp_root and "grievous_stream_" in str(temp_root):
            try:
                logging.info(f"Cleaning up temporary dataset directory: {temp_root}")
                shutil.rmtree(temp_root, ignore_errors=True)
            except Exception as e:
                logging.warning(f"Failed to cleanup temporary directory {temp_root}: {e}")

    # Only push to hub if saving locally
    if cfg.save_locally and cfg.dataset.push_to_hub and dataset is not None:
        dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)

    safe_log_say("Exiting", cfg.play_sounds)
    return dataset


def main():
    register_third_party_devices()
    grievous_record()


if __name__ == "__main__":
    main()

