#!/bin/bash

lerobot-record \
    --robot.type=grievous_client \
    --robot.remote_ip=192.168.50.148 \
    --robot.cameras='{
        left_wrist: {type: opencv, index_or_path: /dev/video6, width: 640, height: 480, fps: 30},
        right_wrist: {type: opencv, index_or_path: /dev/video8, width: 640, height: 480, fps: 30},
        head: {type: intelrealsense, serial_number_or_name: 032622074046, width: 1280, height: 720, fps: 30}
    }' \
    --teleop.type=grievous_leader \
    --teleop.left_arm_port=/dev/ttyACM3 \
    --teleop.right_arm_port=/dev/ttyACM2 \
    --teleop.remote_ip=192.168.50.148 \
    --teleop.port_zmq_cmd=5555 \
    --teleop.id=grievous_leader \
    --dataset.repo_id='InternetSandwich33/test_record' \
    --dataset.num_episodes=10 \
    --dataset.single_task="Test record" \
    --dataset.episode_time_s=10 \
    --dataset.reset_time_s=10 \
    --dataset.push_to_hub=true \
    --dataset.num_image_writer_threads_per_camera=4 \
    --display_data=true