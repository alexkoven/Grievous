#!/bin/bash

# Unified test script for XLerobot/Grievous bimanual leader-follower control
#
# Hardware: 2x SO101 leader arms + XLerobot/Grievous follower
# Usage: ./run_grievous.sh

# Port Configuration - EDIT THESE TO MATCH YOUR HARDWARE
LEADER_1_PORT="/dev/ttyACM3"
LEADER_2_PORT="/dev/ttyACM2"
FOLLOWER_1_PORT="/dev/ttyACM0"
FOLLOWER_2_PORT="/dev/ttyACM1"

# Robot IDs
XLEROBOT_ID="my_xlerobot"
LEADER_ID="my_bimanual_leader"

# Server IP for remote viewing (edit if needed)
SERVER_IP="192.168.50.148"

# Display menu
echo "======================================"
echo "  XLerobot/Grievous Teleoperation"
echo "======================================"
echo ""
echo "Select mode:"
echo "  1) Grievous teleop of top arms without recording"
echo "  2) Grievous teleop of top arms with recording (ZMQ streaming)"
echo ""
read -p "Enter your choice (1 or 2): " choice
echo ""

case $choice in
    1)
        echo "Starting XLerobot bimanual leader-follower teleoperation..."
        echo ""
        echo "Leader arms:"
        echo "  Leader 1: $LEADER_1_PORT"
        echo "  Leader 2: $LEADER_2_PORT"
        echo ""
        echo "XLerobot follower:"
        echo "  Follower 1: $FOLLOWER_1_PORT"
        echo "  Follower 2: $FOLLOWER_2_PORT"
        echo ""
        echo "Press Ctrl+C to stop"
        echo ""

        # Run lerobot-teleoperate
        lerobot-teleoperate \
            --robot.type=bi_so100_follower \
            --robot.id=$XLEROBOT_ID \
            --robot.left_arm_port=$FOLLOWER_1_PORT \
            --robot.right_arm_port=$FOLLOWER_2_PORT \
            --teleop.type=bi_so100_leader \
            --teleop.id=$LEADER_ID \
            --teleop.left_arm_port=$LEADER_1_PORT \
            --teleop.right_arm_port=$LEADER_2_PORT \
            --display_data=true
        ;;
    
    2)
        echo "======================================"
        echo "  Grievous Remote System"
        echo "======================================"
        echo ""
        echo "Select role:"
        echo "  a) Server (RPi5 - robot control with camera streaming)"
        echo "  b) Client (Viewer - remote observation display)"
        echo ""
        read -p "Enter your choice (a or b): " subchoice
        echo ""
        
        case $subchoice in
            a|A)
                echo "Starting Grievous SERVER (RPi5 side)..."
                echo ""
                echo "Leader arms:"
                echo "  Leader 1: $LEADER_1_PORT"
                echo "  Leader 2: $LEADER_2_PORT"
                echo ""
                echo "Grievous follower:"
                echo "  Follower 1: $FOLLOWER_1_PORT"
                echo "  Follower 2: $FOLLOWER_2_PORT"
                echo ""
                echo "Cameras:"
                echo "  Left wrist: /dev/video6"
                echo "  Right wrist: /dev/video8"
                echo "  Head: 032622074046 (Intel RealSense with depth)"
                echo ""
                echo "Broadcasting on: tcp://*:5555"
                echo ""
                echo "Press Ctrl+C to stop"
                echo ""

                # Run grievous-teleoperate (server)
                grievous-teleoperate \
                    --robot.type=bi_so100_follower \
                    --robot.id=$XLEROBOT_ID \
                    --robot.left_arm_port=$FOLLOWER_1_PORT \
                    --robot.right_arm_port=$FOLLOWER_2_PORT \
                    --robot.cameras='{
    left_wrist: {"type": "opencv", "index_or_path": "/dev/video6", "width": 640, "height": 480, "fps": 30},
    right_wrist: {"type": "opencv", "index_or_path": "/dev/video8", "width": 640, "height": 480, "fps": 30},
    head: {"type": "intelrealsense", "serial_number_or_name": "032622074046", "width": 1280, "height": 720, "fps": 30, "use_depth": true}
        }' \
                    --teleop.type=bi_so100_leader \
                    --teleop.id=$LEADER_ID \
                    --teleop.left_arm_port=$LEADER_1_PORT \
                    --teleop.right_arm_port=$LEADER_2_PORT \
                    --remote_client_address=tcp://*:5555
                ;;
            
            b|B)
                echo "Starting Grievous CLIENT (Viewer)..."
                echo ""
                echo "Connecting to server: tcp://$SERVER_IP:5555"
                echo ""
                echo "Press Ctrl+C to stop"
                echo ""

                # Run grievous-viewer (client)
                grievous-viewer --remote_server_address=tcp://$SERVER_IP:5555
                ;;
            
            *)
                echo "Invalid choice. Please run the script again and select 'a' or 'b'."
                exit 1
                ;;
        esac
        ;;
    
    *)
        echo "Invalid choice. Please run the script again and select 1 or 2."
        exit 1
        ;;
esac

