# Changelog

## [grievous-record] - 2025-10-28

### Added
- **grievous_teleoperate.py** - Enhanced teleoperation script with ZMQ remote data streaming
  - Based on `lerobot_teleoperate.py` with added ZMQ connection to send robot data to remote computers
  - Fixed Raspberry Pi RAM issues by removing display options from teleoperation
  - Implemented non-blocking ZMQ sends to prevent teleoperation loop stalling

- **grievous_viewer.py** - Remote observation viewer with Rerun visualization
  - Initializes Rerun session on remote computer
  - Connects to Raspberry Pi via ZMQ to receive and visualize robot logging data

- **test_grievous.sh** - Test script for grievous_teleoperate.py
  - Based on `test_xlerobot.sh` to run the new grievous_teleoperate.py script

### Changed
- **pyproject.toml** - Added CLI command entries
  - Made `grievous_teleoperate.py` and `grievous_viewer.py` executable via command line