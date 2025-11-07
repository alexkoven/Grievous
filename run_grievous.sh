#!/bin/bash
# Grievous Production Control Script
# Manages the Grievous host daemon on RPi5

set -e  # Exit on error

# Configuration
LOG_FILE="/tmp/grievous_host.log"
PID_FILE="/tmp/grievous_host.pid"
ROBOT_ID="grievous_robot"

# Color codes for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Function to check if host is running
is_running() {
    if [ -f "$PID_FILE" ]; then
        pid=$(cat "$PID_FILE")
        if ps -p "$pid" > /dev/null 2>&1; then
            return 0  # Running
        else
            rm -f "$PID_FILE"  # Stale PID file
            return 1  # Not running
        fi
    fi
    return 1  # Not running
}

# Function to get RPi5 IP address
get_ip() {
    hostname -I | awk '{print $1}'
}

# Function to start host daemon
start_host() {
    if is_running; then
        echo -e "${YELLOW}⚠️  Grievous host is already running (PID: $(cat $PID_FILE))${NC}"
        echo "Use option 2 to stop it first."
        return 1
    fi
    
    echo "============================================================"
    echo "Starting Grievous Host Daemon"
    echo "============================================================"
    echo ""
    echo "Configuration:"
    echo "  - Robot ID: $ROBOT_ID"
    echo "  - Calibration: Using saved calibration from cache"
    echo "  - Command port: 5555 (ZMQ PULL socket)"
    echo "  - Observation port: 5556 (ZMQ PUSH socket)"
    echo "  - Watchdog timeout: 500ms"
    echo "  - Max loop frequency: 30 Hz"
    echo ""
    echo "RPi5 IP Address: $(get_ip)"
    echo ""
    read -p "Press ENTER to start daemon..."
    
    # Start daemon in background
    echo ""
    echo "[INFO] Starting Grievous host daemon..."
    nohup python3 -m lerobot.robots.grievous.grievous_host > "$LOG_FILE" 2>&1 &
    echo $! > "$PID_FILE"
    
    # Wait a moment and check if it started successfully
    sleep 2
    if is_running; then
        echo -e "${GREEN}✅ Grievous host daemon started successfully${NC}"
        echo ""
        echo "Process ID: $(cat $PID_FILE)"
        echo "Log file: $LOG_FILE"
        echo ""
        echo "Clients can connect to: $(get_ip):5555 (commands) / $(get_ip):5556 (observations)"
        echo ""
        echo "Use option 2 to stop the daemon."
        echo "Use option 4 to view logs."
    else
        echo -e "${RED}❌ Failed to start host daemon${NC}"
        echo "Check logs: tail -f $LOG_FILE"
        return 1
    fi
}

# Function to stop host daemon
stop_host() {
    if ! is_running; then
        echo -e "${YELLOW}⚠️  Grievous host is not running${NC}"
        return 0
    fi
    
    echo "============================================================"
    echo "Stopping Grievous Host Daemon"
    echo "============================================================"
    echo ""
    pid=$(cat "$PID_FILE")
    echo "Stopping process (PID: $pid)..."
    
    kill "$pid" 2>/dev/null || true
    sleep 1
    
    # Force kill if still running
    if ps -p "$pid" > /dev/null 2>&1; then
        echo "Force stopping..."
        kill -9 "$pid" 2>/dev/null || true
    fi
    
    rm -f "$PID_FILE"
    echo -e "${GREEN}✅ Grievous host daemon stopped${NC}"
}

# Function to check host status
check_status() {
    echo "============================================================"
    echo "Grievous Host Status"
    echo "============================================================"
    echo ""
    
    if is_running; then
        pid=$(cat "$PID_FILE")
        echo -e "${GREEN}✅ Running${NC}"
        echo ""
        echo "Process ID: $pid"
        echo "Log file: $LOG_FILE"
        echo "RPi5 IP: $(get_ip)"
        echo ""
        echo "Connections:"
        echo "  - Commands:     tcp://$(get_ip):5555 (ZMQ PULL)"
        echo "  - Observations: tcp://$(get_ip):5556 (ZMQ PUSH)"
        echo ""
        echo "Recent activity (last 10 lines):"
        echo "---"
        tail -10 "$LOG_FILE" 2>/dev/null || echo "No log data available"
    else
        echo -e "${RED}❌ Not running${NC}"
        echo ""
        if [ -f "$LOG_FILE" ]; then
            echo "Last log entries:"
            echo "---"
            tail -10 "$LOG_FILE"
        fi
    fi
}

# Function to view logs
view_logs() {
    echo "============================================================"
    echo "Grievous Host Logs"
    echo "============================================================"
    echo ""
    echo "Watching: $LOG_FILE"
    echo "Press Ctrl+C to exit"
    echo ""
    
    if [ ! -f "$LOG_FILE" ]; then
        echo "Log file does not exist yet. Start the host daemon first."
        return 1
    fi
    
    tail -f "$LOG_FILE"
}

# Function to calibrate robot
calibrate_robot() {
    if is_running; then
        echo -e "${YELLOW}⚠️  Please stop the host daemon before calibrating${NC}"
        echo "Use option 2 to stop it."
        return 1
    fi
    
    echo "============================================================"
    echo "Calibrate Grievous Robot"
    echo "============================================================"
    echo ""
    echo "This will run interactive calibration for:"
    echo "  - XLerobot (follower arms, base, head)"
    echo "  - BiSO100Leader (leader arms)"
    echo ""
    echo "Robot ID: $ROBOT_ID"
    echo ""
    echo "Calibration Files Location:"
    echo "  ~/.cache/huggingface/lerobot/calibration/"
    echo ""
    echo "Expected calibration files (organized by robot type):"
    echo "  - robots/xlerobot/${ROBOT_ID}_xlerobot.json"
    echo "  - teleoperators/so100_leader/${ROBOT_ID}_leader_left.json"
    echo "  - teleoperators/so100_leader/${ROBOT_ID}_leader_right.json"
    echo ""
    echo "Note: Calibrations are stored by ROBOT TYPE (xlerobot, so100_leader),"
    echo "      not by Grievous. This is correct - Grievous is a composite robot."
    echo ""
    
    # Check for existing calibrations
    CALIB_DIR="$HOME/.cache/huggingface/lerobot/calibration"
    XLEROBOT_CALIB="$CALIB_DIR/robots/xlerobot/${ROBOT_ID}_xlerobot.json"
    LEADER_LEFT_CALIB="$CALIB_DIR/teleoperators/so100_leader/${ROBOT_ID}_leader_left.json"
    LEADER_RIGHT_CALIB="$CALIB_DIR/teleoperators/so100_leader/${ROBOT_ID}_leader_right.json"
    
    existing_files=()
    [ -f "$XLEROBOT_CALIB" ] && existing_files+=("  ✓ XLerobot: $XLEROBOT_CALIB")
    [ -f "$LEADER_LEFT_CALIB" ] && existing_files+=("  ✓ Leader Left: $LEADER_LEFT_CALIB")
    [ -f "$LEADER_RIGHT_CALIB" ] && existing_files+=("  ✓ Leader Right: $LEADER_RIGHT_CALIB")
    
    if [ ${#existing_files[@]} -gt 0 ]; then
        echo -e "${YELLOW}⚠️  Existing calibration files found:${NC}"
        for file in "${existing_files[@]}"; do
            echo "$file"
        done
        echo ""
        echo "Proceeding will OVERWRITE these calibrations."
        echo ""
        read -p "Do you want to continue and overwrite? [y/N]: " confirm
        
        if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
            echo "Calibration cancelled."
            return 0
        fi
    else
        echo -e "${GREEN}✓ No existing calibrations found for ID: $ROBOT_ID${NC}"
        echo "  New calibration files will be created."
    fi
    
    echo ""
    read -p "Press ENTER to start calibration..."
    
    python3 test_grievous_hardware.py
}

# Function to test lerobot-teleoperate
test_lerobot_teleoperate() {
    if is_running; then
        echo -e "${YELLOW}⚠️  Please stop the host daemon before testing${NC}"
        echo "This test will use local hardware directly."
        echo "Use option 2 to stop the daemon."
        return 1
    fi
    
    echo "============================================================"
    echo "Test lerobot-teleoperate (EXPERIMENTAL)"
    echo "============================================================"
    echo ""
    echo "This will test if the standard lerobot-teleoperate command"
    echo "can work with Grievous in the current architecture."
    echo ""
    echo "Expected behavior:"
    echo "  - Robot: Grievous (includes follower + leader in observations)"
    echo "  - Teleop: BiSO100Leader (separate leader arm connection)"
    echo ""
    echo -e "${YELLOW}⚠️  KNOWN ISSUE:${NC}"
    echo "  This will likely FAIL because:"
    echo "  1. Grievous already includes leader arms internally"
    echo "  2. BiSO100Leader will try to connect to same hardware"
    echo "  3. USB device conflict (can't open /dev/ttyACM2-3 twice)"
    echo ""
    echo "This test is to observe the error and understand the limitation."
    echo ""
    read -p "Press ENTER to continue with the test..."
    
    echo ""
    echo "[INFO] Running lerobot-teleoperate with:"
    echo "  --robot.type=grievous"
    echo "  --robot.id=$ROBOT_ID"
    echo "  --teleop.type=bi_so100_leader"
    echo "  --teleop.id=${ROBOT_ID}_leader"
    echo "  --teleop.left_arm_port=/dev/ttyACM3"
    echo "  --teleop.right_arm_port=/dev/ttyACM2"
    echo "  --display_data=true"
    echo ""
    
    lerobot-teleoperate \
        --robot.type=grievous \
        --robot.id=$ROBOT_ID \
        --teleop.type=bi_so100_leader \
        --teleop.id=${ROBOT_ID}_leader \
        --teleop.left_arm_port=/dev/ttyACM3 \
        --teleop.right_arm_port=/dev/ttyACM2 \
        --display_data=true
    
    echo ""
    echo "Test completed (or failed as expected)."
}

# Main menu
echo "============================================================"
echo "Grievous Control Menu (RPi5)"
echo "============================================================"
echo ""
echo "Current Status: $(is_running && echo -e "${GREEN}Running${NC}" || echo -e "${RED}Stopped${NC}")"
echo "RPi5 IP: $(get_ip)"
echo ""
echo "Select an option:"
echo ""
echo "  1) Start Host Daemon"
echo "     - Starts Grievous on RPi5"
echo "     - Listens for network clients"
echo "     - Streams observations and receives actions"
echo ""
echo "  2) Stop Host Daemon"
echo "     - Gracefully stops the running daemon"
echo "     - Disconnects robot hardware"
echo ""
echo "  3) Check Status"
echo "     - View daemon status"
echo "     - Show connection info"
echo "     - Display recent logs"
echo ""
echo "  4) View Logs (live)"
echo "     - Tail the log file in real-time"
echo "     - Press Ctrl+C to exit log viewer"
echo ""
echo "  5) Calibrate Robot"
echo "     - Run interactive calibration"
echo "     - Required before first use"
echo ""
echo "  6) Test lerobot-teleoperate (LOCAL)"
echo "     - EXPERIMENTAL: Test standard lerobot-teleoperate"
echo "     - Uses local hardware (not network)"
echo "     - May fail due to hardware conflicts"
echo ""
echo "  q) Quit"
echo ""
read -p "Enter choice [1/2/3/4/5/6/q]: " choice

case $choice in
    1)
        start_host
        ;;
    2)
        stop_host
        ;;
    3)
        check_status
        ;;
    4)
        view_logs
        ;;
    5)
        calibrate_robot
        ;;
    6)
        test_lerobot_teleoperate
        ;;
    q|Q)
        echo "Exiting..."
        exit 0
        ;;
    *)
        echo "Invalid choice. Please run again and select 1, 2, 3, 4, 5, 6, or q."
        exit 1
        ;;
esac
