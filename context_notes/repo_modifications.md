# Team Modifications to LeRobot Repository

## Recent Commits Summary

This document tracks the files modified by our team during the xLeRobot integration into LeRobot. XLeRobot is a cheap bimanual testbed with a mobile base and two SO101 follower and leader arm pairs. Other files in this repo were created by Huggingface.

---

### Commit 1: ca03d7e - alexkoven
**Message**: "fixed errors, works now"

**Modified Files**:
- `examples/4_xlerobot_teleop_keyboard.py`
- `src/lerobot/robots/xlerobot/xlerobot_client.py`

---

### Commit 2: 1fede42 - Jack
**Message**: "Updated client files"

**Modified Files**:
- `examples/4_xlerobot_teleop_keyboard.py`
- `src/lerobot/robots/xlerobot/xlerobot_client.py`

---

### Commit 3: 37f3b5b - Droneconia
**Message**: "xlerobot integrated into lerobot"

**Added Files**:
- `examples/4_xlerobot_teleop_keyboard.py`
- `src/lerobot/model/SO101Robot.py`
- `src/lerobot/robots/xlerobot/__init__.py`
- `src/lerobot/robots/xlerobot/config_xlerobot.py`
- `src/lerobot/robots/xlerobot/xlerobot.py`
- `src/lerobot/robots/xlerobot/xlerobot_client.py`
- `src/lerobot/robots/xlerobot/xlerobot_host.py`

**Modified Files**:
- `src/lerobot/motors/motors_bus.py`

---

## Running XLerobot Bimanual Leader-Follower

### Quick Start

**Script:** `run_xlerobot.sh` (in workspace root)

```bash
# Edit ports in the script if needed, then run:
./run_xlerobot.sh
```

### What It Does
- Connects to 2x SO101 leader arms via `BiSO100Leader` (SO100 and SO101 are hardware compatible!)
- Connects to XLerobot follower
- Mirrors leader arm positions to follower arms
- Keeps base and head stationary
- 30 Hz control loop with Rerun visualization
- Uses `use_degrees=False` (normalized mode, matches your current config)

### Hardware Setup
Edit these ports in `run_xlerobot.sh`:
- `LEFT_LEADER_PORT="/dev/ttyUSB0"`
- `RIGHT_LEADER_PORT="/dev/ttyUSB1"`
- `XLEROBOT_PORT1="/dev/ttyACM0"`
- `XLEROBOT_PORT2="/dev/ttyACM1"`

### Key Discovery
**SO100 and SO101 are functionally identical!** The only difference is SO101 has a configurable `use_degrees` parameter. This means you can use `BiSO100Leader` directly with SO101 hardware - no need to create separate BiSO101 classes.

---

## Grievous-Record Branch Modifications

This branch adds remote teleoperation and recording capabilities with camera streaming.

### Commit 4: c023be0 - Droneconia
**Message**: "first teleop with record"

**Added Files**:
- `src/lerobot/scripts/grievous_teleoperate.py` - Remote teleoperation script with ZMQ
- `src/lerobot/scripts/grievous_viewer.py` - Viewer for remote observation streams
- `test_grievous.sh` - Test script for grievous teleoperation

**Modified Files**:
- `pyproject.toml`

---

### Commit 5: 7b5b280 - Droneconia
**Message**: "RAM issue fixed, ZMQ noblock added"

**Added Files**:
- `CHANGELOG.md` - Changelog for tracking updates

**Modified Files**:
- `src/lerobot/scripts/grievous_teleoperate.py`
- `test_grievous.sh`

---

### Commit 6: 1d6aa3b - Droneconia
**Message**: "Added Camera data streaming"

**Modified Files**:
- `src/lerobot/scripts/grievous_teleoperate.py` - Added camera streaming support
- `src/lerobot/scripts/grievous_viewer.py` - Added camera display
- `test_grievous.sh`

---

### Commit 7: 5b4ce60 - Droneconia
**Message**: "FPS fix in progress"

**Modified Files**:
- `CHANGELOG.md`
- `src/lerobot/robots/bi_so100_follower/bi_so100_follower.py` - FPS optimizations
- `src/lerobot/scripts/grievous_teleoperate.py`

---

### Commit 8: 9480be3 - Droneconia
**Message**: "FPS issue fixed, viewer added"

**Added Files**:
- `test_viewer.sh` - Standalone viewer test script

**Modified Files**:
- `CHANGELOG.md`
- `src/lerobot/robots/bi_so100_follower/bi_so100_follower.py` - Final FPS fixes
- `src/lerobot/scripts/grievous_teleoperate.py`
- `src/lerobot/scripts/grievous_viewer.py`
- `src/lerobot/utils/visualization_utils.py` - Visualization improvements

---

### Commit 9: e68029c - Droneconia
**Message**: "In progress compression plus docs"

**Modified Files**:
- `CHANGELOG.md`
- `src/lerobot/scripts/grievous_teleoperate.py` - Added compression for data streaming

---

### Commit 10: 7d79414 - Droneconia
**Message**: "Grievous viewer update to decode message"

**Modified Files**:
- `src/lerobot/scripts/grievous_viewer.py` - Message decoding improvements

---

## Summary of All Team Modifications

### Core XLerobot Files (Main Branch)
- `src/lerobot/robots/xlerobot/` - Complete XLerobot implementation
  - `__init__.py`
  - `config_xlerobot.py`
  - `xlerobot.py`
  - `xlerobot_client.py`
  - `xlerobot_host.py`
- `src/lerobot/model/SO101Robot.py` - SO101 kinematics model
- `examples/4_xlerobot_teleop_keyboard.py` - Keyboard teleoperation

### Grievous Remote System (Grievous-Record Branch)
- `src/lerobot/scripts/grievous_teleoperate.py` - Remote teleoperation with ZMQ
- `src/lerobot/scripts/grievous_viewer.py` - Remote observation viewer
- `test_grievous.sh` - Grievous system test script
- `test_viewer.sh` - Viewer test script
- `CHANGELOG.md` - Project changelog

### Modified LeRobot Files
- `src/lerobot/motors/motors_bus.py` - Motor bus modifications
- `src/lerobot/robots/bi_so100_follower/bi_so100_follower.py` - FPS optimizations
- `src/lerobot/utils/visualization_utils.py` - Visualization improvements
- `pyproject.toml` - Project dependencies

### Test Scripts
- `test_xlerobot.sh` - Bimanual leader-follower test
- `test_grievous.sh` - Remote teleoperation test
- `test_viewer.sh` - Viewer test

