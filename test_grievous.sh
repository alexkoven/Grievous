#!/bin/bash
# Grievous Testing Script
# Provides menu-driven testing for Part A (hardware) and Part B (network)

set -e  # Exit on error

# Navigate to workspace
cd /home/xlerobot/Grievous

# Activate conda environment
source /home/xlerobot/miniconda3/bin/activate lerobot

echo "============================================================"
echo "Grievous Testing Menu"
echo "============================================================"
echo ""
echo "Select test to run:"
echo ""
echo "  1) Part A: Hardware Validation (RPi5 local)"
echo "     - Interactive calibration"
echo "     - Test XLerobot + BiSO100Leader"
echo "     - Verify observations"
echo ""
echo "  2) Part B: Network Communication Test"
echo "     2a) Start Host Daemon (RPi5 side)"
echo "     2b) Test Client Connection (laptop side)"
echo ""
echo "  q) Quit"
echo ""
read -p "Enter choice [1/2a/2b/q]: " choice

case $choice in
    1)
        echo ""
        echo "============================================================"
        echo "Part A: Hardware Validation"
        echo "============================================================"
        echo ""
        echo "This will:"
        echo "  - Calibrate all motors (you'll position arms manually)"
        echo "  - Test observation retrieval"
        echo "  - Verify leader arm integration"
        echo ""
        read -p "Press ENTER to continue..."
        
        echo ""
        echo "[INFO] Starting hardware test with calibration..."
        python3 test_grievous_hardware.py
        
        echo ""
        echo "============================================================"
        echo "✅ Hardware validation complete!"
        echo "============================================================"
        ;;
        
    2a)
        echo ""
        echo "============================================================"
        echo "Part B: Network Host Daemon (RPi5)"
        echo "============================================================"
        echo ""
        echo "This will:"
        echo "  - Start Grievous host daemon on RPi5"
        echo "  - Listen on ports 5555 (commands) and 5556 (observations)"
        echo "  - Stream observations to network clients"
        echo "  - Receive action commands from clients"
        echo ""
        echo "Press Ctrl+C to stop the daemon when done."
        echo ""
        read -p "Press ENTER to start host daemon..."
        
        echo ""
        echo "[INFO] Getting RPi5 IP address..."
        echo "RPi5 IP: $(hostname -I | awk '{print $1}')"
        echo ""
        echo "[INFO] Starting Grievous host daemon..."
        python3 src/lerobot/robots/grievous/grievous_host.py
        ;;
        
    2b)
        echo ""
        echo "============================================================"
        echo "Part B: Network Client Test (Laptop)"
        echo "============================================================"
        echo ""
        echo "⚠️  IMPORTANT: Run this on your LAPTOP, not on RPi5!"
        echo ""
        echo "This will:"
        echo "  - Connect to Grievous host on RPi5"
        echo "  - Test observation streaming"
        echo "  - Test action sending"
        echo "  - Measure network latency"
        echo ""
        read -p "Enter RPi5 IP address: " rpi5_ip
        
        if [ -z "$rpi5_ip" ]; then
            echo "ERROR: IP address cannot be empty"
            exit 1
        fi
        
        echo ""
        echo "[INFO] Testing client connection to $rpi5_ip..."
        python3 -c "
from lerobot.robots.grievous import GrievousClient, GrievousClientConfig
import time

print('=' * 60)
print('Grievous Client Test')
print('=' * 60)

# Create client
config = GrievousClientConfig(remote_ip='$rpi5_ip')
client = GrievousClient(config)

# Test connection
print('\n[1/4] Connecting to host...')
try:
    client.connect()
    print('✅ Connected to Grievous host at $rpi5_ip')
except Exception as e:
    print(f'❌ Connection failed: {e}')
    exit(1)

# Test observation retrieval
print('\n[2/4] Testing observation retrieval...')
obs = client.get_observation()
print(f'✅ Got {len(obs)} observation keys')

# Analyze observations
follower_keys = [k for k in obs.keys() if '_leader' not in k and k not in ['left_wrist', 'right_wrist', 'head']]
leader_keys = [k for k in obs.keys() if '_leader' in k]
camera_keys = [k for k in obs.keys() if k in ['left_wrist', 'right_wrist', 'head']]
print(f'   Follower states: {len(follower_keys)}')
print(f'   Leader states: {len(leader_keys)}')
print(f'   Cameras: {len(camera_keys)}')

# Test observation update rate
print('\n[3/4] Testing observation update rate...')
start = time.time()
for i in range(30):
    obs = client.get_observation()
elapsed = time.time() - start
hz = 30 / elapsed
print(f'✅ 30 observations in {elapsed:.2f}s = {hz:.1f} Hz')

# Test action sending
print('\n[4/4] Testing action sending...')
action = {k: 0.0 for k in client.action_features.keys()}
client.send_action(action)
print('✅ Action sent successfully')

# Disconnect
client.disconnect()
print('\n' + '=' * 60)
print('✅ All client tests PASSED!')
print('=' * 60)
"
        ;;
        
    q|Q)
        echo "Exiting..."
        exit 0
        ;;
        
    *)
        echo "Invalid choice. Please run again and select 1, 2a, 2b, or q."
        exit 1
        ;;
esac

