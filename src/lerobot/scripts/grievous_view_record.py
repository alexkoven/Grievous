# COPIED: Copyright header from lerobot scripts
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
Remote viewer and dataset recorder for grievous_teleoperate/grievous_record observations via ZMQ.

This script connects to a running grievous_teleoperate or grievous_record instance,
displays the robot observations in Rerun, and records the dataset locally.

Usage:

```shell
grievous-view-record \
    --remote_server_address=tcp://192.168.1.100:5555 \
    --dataset.repo_id=<my_username>/<my_dataset_name> \
    --dataset.single_task="Grab the cube"
```

"""

# COPIED: Standard imports from lerobot scripts
import logging
import pickle
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import zmq
import numpy as np
import base64
import cv2

# COPIED: Rerun import from lerobot_teleoperate.py
import rerun as rr

# COPIED: Config and processor imports from lerobot scripts
from lerobot.configs import parser
from lerobot.datasets.image_writer import safe_stop_image_writer
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.pipeline_features import aggregate_pipeline_dataset_features, create_initial_features
from lerobot.datasets.utils import combine_feature_dicts
from lerobot.datasets.video_utils import VideoEncodingManager
from lerobot.processor import (
    RobotObservation,
    RobotProcessorPipeline,
    make_default_processors,
)
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.control_utils import (
    init_keyboard_listener,
    is_headless,
    sanity_check_dataset_name,
)
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data


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
    episode_time_s: int | float = 60
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
    # Display all cameras on screen
    display_data: bool = False

    def __post_init__(self):
        if self.single_task is None:
            raise ValueError("You need to provide a task as argument in `single_task`.")


@dataclass
class GrievousViewRecordConfig:
    # Remote server address to connect to (e.g., "tcp://192.168.1.100:5555")
    remote_server_address: str
    # Dataset configuration
    dataset: DatasetRecordConfig
    # Name for the rerun session
    session_name: str = "grievous_remote_viewer_recorder"
    # Camera list for display (cameras not in this list won't be decoded for display)
    cam_list: list = field(default_factory=lambda: ["head", "left_wrist", "right_wrist"])
    # Use vocal synthesis to read events.
    play_sounds: bool = True


# COPIED: Image decoding function from xlerobot_client.py (lines 186-199)
# FIXED: Removed 'self' parameter as this is not a class method
def _decode_image_from_b64(image_b64: str) -> Optional[np.ndarray]:
    """Decodes a base64 encoded image string to an OpenCV image."""
    if not image_b64:
        return None
    try:
        jpg_data = base64.b64decode(image_b64)
        np_arr = np.frombuffer(jpg_data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            logging.warning("cv2.imdecode returned None for an image.")
        return frame
    except (TypeError, ValueError) as e:
        logging.error(f"Error decoding base64 image data: {e}")
        return None


@safe_stop_image_writer
def record_viewer_loop(
    zmq_socket: zmq.Socket,
    dataset: LeRobotDataset,
    cfg: GrievousViewRecordConfig,
    robot_observation_processor: RobotProcessorPipeline[RobotObservation, RobotObservation],
    events: dict,
    control_time_s: float | None = None,
):
    """
    Main loop that receives observations via ZMQ, displays them in Rerun, and records to dataset.
    Similar to lerobot_record.py's record_loop but receives data over ZMQ instead of directly from robot.
    
    Args:
        zmq_socket: ZMQ socket connected to the remote server
        dataset: LeRobotDataset instance for recording
        cfg: Configuration for the viewer/recorder
        robot_observation_processor: Processor for robot observations
        events: Keyboard events dictionary for controlling recording
        control_time_s: Duration to record for this episode (None = use cfg.dataset.episode_time_s)
    """
    if control_time_s is None:
        control_time_s = cfg.dataset.episode_time_s
    
    timestamp = 0
    start_episode_t = time.perf_counter()
    
    while timestamp < control_time_s:
        start_loop_t = time.perf_counter()
        
        if events["exit_early"]:
            events["exit_early"] = False
            break
        
        # Receive observation data (using pickle as in grievous_record.py)
        try:
            data = zmq_socket.recv(zmq.NOBLOCK)
            obs_data = pickle.loads(data)
        except zmq.Again:
            # No data available yet, wait a bit and continue
            time.sleep(0.001)
            timestamp = time.perf_counter() - start_episode_t
            continue
        
        # Extract observation and action
        obs = obs_data.get("observation", {})
        action = obs_data.get("action", {})
        frame_timestamp = obs_data.get("timestamp", time.time())
        
        # Extract dataset frame (already properly formatted from server)
        dataset_frame = obs_data.get("dataset_frame")
        if dataset_frame is not None:
            # Add frame to dataset (same as lerobot_record.py line 361)
            try:
                dataset.add_frame(dataset_frame)
            except Exception as e:
                logging.error(f"Failed to add frame to dataset: {e}")
        
        # Decode base64 images for specified cameras (for display only)
        # The dataset_frame already has numpy arrays, so we only decode for display
        display_obs = obs.copy()
        for cam_name, image in obs.items():
            if cam_name in cfg.cam_list and isinstance(image, str):
                # Only decode if it's a base64 string (for display)
                frame = _decode_image_from_b64(image)
                if frame is not None:
                    display_obs[cam_name] = frame
        
        # Process observation for display (same as lerobot_record.py)
        obs_processed = robot_observation_processor(display_obs)
        
        # Log to Rerun (same as lerobot_record.py line 364)
        # Rerun is always initialized in grievous_view_record, so always log
        log_rerun_data(observation=obs_processed, action=action)
        
        dt_s = time.perf_counter() - start_loop_t
        # Wait to maintain FPS (similar to lerobot_record.py but we're receiving not generating)
        # Use a small sleep to avoid busy-waiting
        time.sleep(max(0, 1.0 / dataset.fps - dt_s))
        
        timestamp = time.perf_counter() - start_episode_t


# NEW: Main viewer/recorder function with ZMQ client, Rerun visualization, and dataset recording
@parser.wrap()
def grievous_view_record(cfg: GrievousViewRecordConfig):
    # COPIED: Logging initialization from lerobot scripts
    init_logging()
    logging.info(f"Connecting to {cfg.remote_server_address}")
    logging.info(f"Will record dataset: {cfg.dataset.repo_id}")
    
    # COPIED: Rerun initialization from lerobot_teleoperate.py (lines 188-189)
    # Initialize Rerun
    init_rerun(session_name=cfg.session_name)
    
    # COPIED: ZMQ connection pattern from xlerobot_client.py (lines 128-137)
    # Set up ZMQ connection
    zmq_context = zmq.Context()
    zmq_socket = zmq_context.socket(zmq.PULL)
    zmq_socket.connect(cfg.remote_server_address)
    
    logging.info("Connected! Receiving observations... Press Ctrl+C to stop.")
    
    # NEW: Get processors
    _, _, robot_observation_processor = make_default_processors()
    
    # NEW: Wait for first message to get dataset features and fps
    # This allows us to create the dataset with the correct features from the remote server
    logging.info("Waiting for first observation to get dataset features...")
    dataset = None
    dataset_features = None
    dataset_fps = None
    first_message_received = False
    
    while not first_message_received:
        try:
            data = zmq_socket.recv(zmq.NOBLOCK)
            obs_data = pickle.loads(data)
            
            # Try to get dataset features and fps from the message
            dataset_features = obs_data.get("dataset_features")
            dataset_fps = obs_data.get("dataset_fps")
            
            if dataset_features is not None and dataset_fps is not None:
                logging.info(f"Received dataset features and fps={dataset_fps} from remote server")
                # Create dataset with received features
                sanity_check_dataset_name(cfg.dataset.repo_id, None)
                dataset = LeRobotDataset.create(
                    cfg.dataset.repo_id,
                    dataset_fps,
                    root=cfg.dataset.root,
                    robot_type=None,  # We don't know the robot type on the client side
                    features=dataset_features,
                    use_videos=cfg.dataset.video,
                    image_writer_processes=cfg.dataset.num_image_writer_processes,
                    image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * len(obs_data.get("observation", {})),
                    batch_encoding_size=cfg.dataset.video_encoding_batch_size,
                )
                first_message_received = True
                logging.info(f"Created dataset with features: {list(dataset_features.keys())}")
            else:
                # If no dataset features in message, create a default dataset
                # This might happen if connecting to grievous_teleoperate (not grievous_record)
                logging.warning("No dataset features in message. Creating default dataset structure.")
                # Use default features based on observation structure
                # This is a fallback - ideally we should always receive dataset_features
                first_message_received = True
                
        except zmq.Again:
            time.sleep(0.1)
            continue
    
    if dataset is None:
        raise ValueError("Failed to create dataset. Make sure the remote server is sending dataset_features.")
    
    # Initialize keyboard listener for recording control (same as lerobot_record.py)
    listener, events = init_keyboard_listener()
    
    # Main recording/viewing loop with VideoEncodingManager (same as lerobot_record.py)
    with VideoEncodingManager(dataset):
        recorded_episodes = 0
        while recorded_episodes < cfg.dataset.num_episodes and not events["stop_recording"]:
            log_say(f"Recording episode {dataset.num_episodes}", cfg.play_sounds)
            record_viewer_loop(
                zmq_socket=zmq_socket,
                dataset=dataset,
                cfg=cfg,
                robot_observation_processor=robot_observation_processor,
                events=events,
                control_time_s=cfg.dataset.episode_time_s,
            )
            
            # Execute reset period without recording (same as lerobot_record.py)
            # Skip reset for the last episode to be recorded
            if not events["stop_recording"] and (
                (recorded_episodes < cfg.dataset.num_episodes - 1) or events["rerecord_episode"]
            ):
                log_say("Reset the environment", cfg.play_sounds)
                record_viewer_loop(
                    zmq_socket=zmq_socket,
                    dataset=dataset,
                    cfg=cfg,
                    robot_observation_processor=robot_observation_processor,
                    events=events,
                    control_time_s=cfg.dataset.reset_time_s,
                )
            
            if events["rerecord_episode"]:
                log_say("Re-record episode", cfg.play_sounds)
                events["rerecord_episode"] = False
                events["exit_early"] = False
                if dataset is not None:
                    dataset.clear_episode_buffer()
                continue
            
            # Save episode (same as lerobot_record.py line 495)
            if dataset is not None:
                dataset.save_episode()
            recorded_episodes += 1
    
    log_say("Stop recording", cfg.play_sounds, blocking=True)
    
    # Cleanup (same as lerobot_record.py)
    if not is_headless() and listener is not None:
        listener.stop()
    
    # Push to hub (same as lerobot_record.py line 507-508)
    if cfg.dataset.push_to_hub and dataset is not None:
        try:
            logging.info(f"Pushing dataset to Hugging Face Hub: {cfg.dataset.repo_id}")
            dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)
            logging.info(f"Successfully pushed dataset to Hugging Face Hub: https://huggingface.co/datasets/{cfg.dataset.repo_id}")
        except Exception as e:
            error_msg = str(e)
            if "401" in error_msg or "Unauthorized" in error_msg or "authentication" in error_msg.lower():
                logging.error(
                    "Failed to push dataset to Hugging Face Hub: Authentication required.\n"
                    "To fix this, you need to authenticate with Hugging Face:\n"
                    "  1. Get your write token from: https://huggingface.co/settings/tokens\n"
                    "  2. Run: huggingface-cli login --token YOUR_TOKEN\n"
                    "     Or: huggingface-cli login (to login interactively)\n"
                    f"Your dataset is saved locally at: {dataset.root}\n"
                    "You can push it manually later using: dataset.push_to_hub()"
                )
            elif "403" in error_msg or "Forbidden" in error_msg:
                logging.error(
                    f"403 Forbidden: Permission denied for repository '{cfg.dataset.repo_id}'.\n"
                    "Check that:\n"
                    "  1. Your token has write permissions\n"
                    "  2. The repo_id username matches your authenticated username\n"
                    f"Your dataset is saved locally at: {dataset.root}"
                )
            else:
                logging.error(f"Failed to push dataset to Hugging Face Hub: {e}")
                logging.info(f"Your dataset is saved locally at: {dataset.root}")
    
    log_say("Exiting", cfg.play_sounds)
    
    # Cleanup ZMQ and Rerun resources
    zmq_socket.close()
    zmq_context.term()
    rr.rerun_shutdown()


# COPIED: main() entry point pattern from lerobot scripts
def main():
    grievous_view_record()


if __name__ == "__main__":
    main()

