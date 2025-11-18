#!/usr/bin/env python

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
Calibration diagnostics utility for robot arms.

This module provides detailed logging when calibration mismatches are detected,
helping users understand and resolve calibration issues.
"""

import logging
from pathlib import Path

from lerobot.motors import MotorCalibration
from lerobot.motors.motors_bus import MotorsBus

logger = logging.getLogger(__name__)


def log_calibration_mismatch(
    robot_id: str,
    calibration_fpath: Path,
    calibration: dict[str, MotorCalibration],
    motor_bus: MotorsBus,
) -> None:
    """
    Log detailed information about calibration mismatch between file and motors.
    
    This function provides a comprehensive diagnostic output showing:
    - Whether the calibration file exists
    - Values loaded from the calibration file
    - Values read from the physical motors
    - Exact differences between file and motor values
    - Recommended next steps
    
    Args:
        robot_id: The robot identifier (e.g., "my_xlerobot_left")
        calibration_fpath: Path to the calibration JSON file
        calibration: Dictionary of calibration data loaded from file
        motor_bus: The motor bus instance to read motor calibration from
        
    Example output:
        ================================================================================
        CALIBRATION MISMATCH DETECTED for my_xlerobot_left
        ================================================================================
        
        1. Calibration file path: /path/to/my_xlerobot_left.json
           ✓ File exists
        
        2. Calibration loaded from file:
           • shoulder_pan: homing_offset=-1223, range=[930, 3466]
           ...
        
        3. Calibration read from motors:
           • shoulder_pan: homing_offset=-1200, range=[930, 3466]
           ...
        
        4. Detected differences:
           • shoulder_pan:
             - homing_offset: -1223 (file) vs -1200 (motor)
        
        5. Next steps:
           - Press ENTER to use the calibration file values (recommended)
           - Type 'c' + ENTER to run full calibration from scratch
        ================================================================================
    """
    logger.info("\n" + "=" * 80)
    logger.info(f"CALIBRATION MISMATCH DETECTED for {robot_id}")
    logger.info("=" * 80)
    
    # Check if calibration file exists
    logger.info(f"\n1. Calibration file path: {calibration_fpath}")
    if not calibration_fpath.exists():
        logger.info("   ❌ No calibration file found")
        logger.info("   → A new calibration will be created")
    else:
        logger.info(f"   ✓ File exists")
        logger.info(f"\n2. Calibration loaded from file:")
        if calibration:
            for motor_name, motor_cal in calibration.items():
                logger.info(f"   • {motor_name}: "
                          f"homing_offset={motor_cal.homing_offset}, "
                          f"range=[{motor_cal.range_min}, {motor_cal.range_max}]")
        else:
            logger.info("   ⚠️  No calibration loaded (file may be empty or corrupted)")
        
        # Read actual motor values
        logger.info(f"\n3. Calibration read from motors:")
        try:
            motor_calibration = motor_bus.read_calibration()
            for motor_name, motor_cal in motor_calibration.items():
                logger.info(f"   • {motor_name}: "
                          f"homing_offset={motor_cal.homing_offset}, "
                          f"range=[{motor_cal.range_min}, {motor_cal.range_max}]")
            
            # Show differences
            if calibration:
                logger.info(f"\n4. Detected differences:")
                has_diff = False
                for motor_name in calibration.keys():
                    file_cal = calibration[motor_name]
                    motor_cal = motor_calibration.get(motor_name)
                    if motor_cal:
                        diffs = []
                        if file_cal.homing_offset != motor_cal.homing_offset:
                            diffs.append(f"homing_offset: {file_cal.homing_offset} (file) vs {motor_cal.homing_offset} (motor)")
                        if file_cal.range_min != motor_cal.range_min:
                            diffs.append(f"range_min: {file_cal.range_min} (file) vs {motor_cal.range_min} (motor)")
                        if file_cal.range_max != motor_cal.range_max:
                            diffs.append(f"range_max: {file_cal.range_max} (file) vs {motor_cal.range_max} (motor)")
                        
                        if diffs:
                            has_diff = True
                            logger.info(f"   • {motor_name}:")
                            for diff in diffs:
                                logger.info(f"     - {diff}")
                
                if not has_diff:
                    logger.info("   ⚠️  No differences found, but is_calibrated returned False")
                    logger.info("   → This might indicate a protocol version or comparison logic issue")
                    
        except Exception as e:
            logger.info(f"   ❌ Error reading motor calibration: {e}")
    
    logger.info("\n5. Next steps:")
    logger.info("   - Press ENTER to use the calibration file values (recommended)")
    logger.info("   - Type 'c' + ENTER to run full calibration from scratch")
    logger.info("=" * 80 + "\n")

