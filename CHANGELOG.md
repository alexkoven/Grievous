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

## [grievous-record] - 2025-10-29

### Changed
- **grievous_teleoperate.py**
  - Tried to fix frame rate issue by implementing camera threading, but the thread was not actually taking the camera load from the script.
  - Tried to offload observation to a thread, but it is interrupted by another script using the serial bus, which causes the program to crash.
  - Next Steps: Make sure the camera script connects and records camera data, then remove camera handling from the BiSO100 robot config we are using, this will get us back to 60fps for control and give us as much frames as we can hold. Reason it does not connect is likely the camera class used, so find a way to change it from the template to OpenCV camera and Intellsense camera.

## [grievous-record] - 2025-10-30

### Changed
- **grievous_teleoperate.py**
  - Combined observation and camera data into one dict to make it compatible with lerobot's visualization utility, visualization_utils.py
  - Edited CameraReader class to use the actual camera, instead of generic camera class
  - Removed jpg compression from CameraReader class
  - Moved camera reader initialization to after robot connection to improve startup time
  - Removed ObservationReader, because it was unused

- **bi_so100_follower.py**
  - Removed camera handling from this class, so that it wouldn't hang up the script

- **grievous_viewer.py**
  - Implemented simple rerun visualization, using lerobot utilities

- **test_viewer.sh**
  - Shell script to initialize rerun visualizer

- **visualization_utils.py**
  - Removed static=TRUE from image logging, so we can record camera data as well

### Next Steps
- Compress image data to reduce recording file size.
- Auto log data into a VLA compatible file
- Make sure Rerun Data can be used for VLA training

## [grievous-record] - 2025-10-30
- **test_grievous.sh**
  - Changed camera resolutions, webcams went from 640x480 to 640x360, Intellsense went from 1280x720 to 424x240
  -

- **grievous_teleoperate.py**
  - Implemented image compression based on XleRobot to reduce recording size and network usage:
  '''python
  frame = cv2.resize(frame, (512,512), interpolation=cv2.INTER_AREA)
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    result, encframe = cv2.imencode('.jpg', frame, encode_param)
    if result:
        camera_data[cam_name] = base64.b64encode(encframe).decode("utf-8")
  '''

- **grievous_viewer.py**
  - Started to implement decoding:
  '''python
  
  '''

## [alex_grievous_record] - 2025-11-03
- **grievous_teleoperate.py**
  - Changed pickle to JSON for sending data over ZMQ

