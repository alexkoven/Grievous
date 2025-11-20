#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

from dataclasses import dataclass, field

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("grievous_leader")
@dataclass
class GrievousLeaderConfig(TeleoperatorConfig):
    """Configuration for Grievous leader arms teleoperator on laptop.
    
    Port configuration (from laptop_host_setup.md):
    - Leader left arm: /dev/ttyACM0
    - Leader right arm: /dev/ttyACM3
    """
    left_arm_port: str = "/dev/ttyACM0"  # Leader left arm
    right_arm_port: str = "/dev/ttyACM3"  # Leader right arm
    remote_ip: str = "192.168.50.148"
    port_zmq_cmd: int = 5555

