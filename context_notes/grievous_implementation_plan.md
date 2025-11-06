# Grievous Robot Implementation Plan

**Status**: Planning Phase  
**Last Updated**: 2025-01-06

---

## Abstract

This document specifies the refactoring of the Grievous robot implementation to align with lerobot's standard architecture patterns. Grievous is a 4-arm teleoperation platform combining an XLerobot base (2 follower arms, omnidirectional mobile base, head motors, RGB-D cameras on RPi5) with 2 SO-100 leader arms for direct physical teleoperation. Current implementation uses custom `grievous_*.py` scripts that duplicate functionality from `lerobot_record.py` and `lerobot_teleoperate.py`, preventing use of standard training/evaluation pipelines and creating maintenance overhead. They have since been deleted. The refactor implements a **composition pattern** where the `Grievous` robot class contains instances of `XLerobot` (follower subsystem) and `BiSO100Leader` (leader subsystem), delegating all operations to these proven components. This mirrors the existing `BiSO100Follower` architecture which wraps two `SO100Follower` instances. The implementation follows the client-host network architecture: `grievous_host.py` runs on RPi5 managing all hardware (reading leader positions, controlling followers, streaming camera feeds), while `grievous_client.py` provides a network-transparent `Robot` interface to the laptop running standard lerobot scripts. Observations include both follower and leader states (with `_leader` suffix) plus camera frames, while actions contain only follower commands (leaders are input-only). This architecture enables immediate use of `lerobot-record`, `lerobot-train`, and `lerobot-eval` without script modifications, reduces codebase from ~2000 lines of custom scripts to ~150 lines of composition logic, ensures automatic propagation of upstream improvements, and provides a template for adding the planned 2 bottom follower arms using the same compositional approach.

---

## What is Grievous?

**Grievous = XLerobot + 2 Leader Arms (+ Future: 2 Bottom Followers)**

- **Current**: XLerobot (2 top follower arms, mobile base, head, cameras) + 2 SO-100 leader arms
- **Future**: Add 2 bottom SO-100 follower arms for ground-level picking
- **Onboard**: Raspberry Pi 5 with all hardware connected via USB
- **Remote**: Laptop/Desktop for recording, visualization, training only

**Key Difference from XLerobot**:
- XLerobot uses VR headset for teleoperation (VR on laptop → actions to robot)
- Grievous uses physical leader arms on robot (leader arms on RPi5 → read locally)
- From laptop's perspective: **identical architecture** (same client/host pattern)

---

## Implementation Strategy: Composition Pattern

**Approach**: `Grievous` class contains `XLerobot` + `BiSO100Leader` instances

**Pattern Reference**: Same as `BiSO100Follower` which wraps two `SO100Follower` instances



**Implementation**:
- `Grievous.__init__()`: Create XLerobot + BiSO100Leader instances
- `Grievous.connect()`: Delegate to both components
- `Grievous.get_observation()`: Query both, merge with prefix handling
- `Grievous.send_action()`: Extract follower subset, send to follower only

---

## State Features Design

**Observations** (what we read):
- All XLerobot state: follower arms, base velocities, head motors
- All BiSO100Leader state: leader arm positions (with `_leader` suffix)
- All camera frames from XLerobot

**Actions** (what we control):
- Only XLerobot state: follower arms, base velocities, head motors
- Leader arms are read-only inputs (not actuated)

**Naming Convention**:
- Follower: `left_arm_shoulder_pan.pos`, `right_arm_shoulder_pan.pos`, etc.
- Leader: `left_leader_shoulder_pan.pos`, `right_leader_shoulder_pan.pos`, etc.
- Pattern: Add `_leader` suffix to motor name (matches BiSO100 prefix pattern)

---

## File Structure

```
src/lerobot/robots/grievous/
├── __init__.py                    # Module exports
├── config_grievous.py             # All 3 config classes
├── grievous.py                    # Composite robot class
├── grievous_host.py               # RPi5 daemon (copy xlerobot_host)
└── grievous_client.py             # Network client (copy xlerobot_client)
```

**Configs Needed**:
1. `GrievousConfig`: Real hardware (registered as `"grievous"`)
2. `GrievousHostConfig`: RPi5 daemon settings (not a RobotConfig)
3. `GrievousClientConfig`: Network client (registered as `"grievous_client"`)

---

## Design Decisions Summary

### 1. Leader-Follower Mapping
Use existing BiSO100Leader/BiSO100Follower mapping (left→left, right→right). No custom transformation needed.

### 2. Recording Strategy
Record everything: observations include follower + leader + cameras. Actions include only follower. Enables data analysis of teleoperation behavior.

### 3. Base & Head Control
Keyboard teleoperation following XLerobot pattern. Reuse existing XLerobot keyboard control logic.

### 4. Calibration
Sequential delegation: `Grievous.calibrate()` calls `follower.calibrate()` then `leader.calibrate()`. Each component handles its own calibration.

---

## Standard Scripts Integration

Once implemented, use standard lerobot scripts:

```bash
# On RPi5:
python src/lerobot/robots/grievous/grievous_host.py

# On Laptop:
lerobot-record --robot.type=grievous_client --robot.remote_ip=<rpi5_ip> ...
lerobot-teleoperate --robot.type=grievous_client --robot.remote_ip=<rpi5_ip> ...
lerobot-train --dataset.repo_id=user/dataset ...
lerobot-eval --robot.type=grievous_client --robot.remote_ip=<rpi5_ip> ...
```

