#!/usr/bin/env python3
"""Interactive hardware test for Grievous robot."""

from lerobot.robots.grievous import Grievous, GrievousConfig

def main():
    print("=" * 60)
    print("Grievous Hardware Validation Test")
    print("=" * 60)
    
    # Create robot with proper ID for calibration management
    print("\n[1/4] Creating Grievous robot...")
    config = GrievousConfig(id="grievous_robot")  # Proper ID to avoid None collisions
    robot = Grievous(config)
    print("✅ Grievous created with ID: grievous_robot")
    
    # Connect with calibration (interactive)
    print("\n[2/4] Connecting and calibrating...")
    print("This will prompt you to position arms manually.")
    robot.connect(calibrate=True)
    print("✅ Connected and calibrated")
    
    # Test observations
    print("\n[3/4] Testing observation retrieval...")
    obs = robot.get_observation()
    print(f"✅ Got {len(obs)} observation keys")
    
    # Analyze observation structure
    print("\n=== Observation Breakdown ===")
    follower_keys = [k for k in obs.keys() if '_leader' not in k and k not in ['left_wrist', 'right_wrist', 'head']]
    leader_keys = [k for k in obs.keys() if '_leader' in k]
    camera_keys = [k for k in obs.keys() if k in ['left_wrist', 'right_wrist', 'head']]
    
    print(f"Follower state keys: {len(follower_keys)}")
    print(f"Leader state keys: {len(leader_keys)}")
    print(f"Camera keys: {len(camera_keys)}")
    
    print(f"\n=== Sample Leader Keys (verify _leader suffix) ===")
    for key in (leader_keys[:5] if len(leader_keys) >= 5 else leader_keys):
        print(f"  - {key}: {obs[key]}")
    
    print(f"\n=== Sample Follower Keys ===")
    for key in (follower_keys[:5] if len(follower_keys) >= 5 else follower_keys):
        print(f"  - {key}: {obs[key]}")
    
    print(f"\n=== Camera Keys ===")
    for key in camera_keys:
        if key in obs:
            img_shape = obs[key].shape if hasattr(obs[key], 'shape') else type(obs[key])
            print(f"  - {key}: {img_shape}")
    
    # Disconnect
    print("\n[4/4] Disconnecting...")
    robot.disconnect()
    print("✅ Disconnected cleanly")
    
    print("\n" + "=" * 60)
    print("✅ Hardware validation PASSED!")
    print("=" * 60)

if __name__ == "__main__":
    main()

