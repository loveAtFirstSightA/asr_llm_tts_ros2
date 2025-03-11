#!/bin/bash

colcon build && source install/local_setup.bash && ros2 launch ifly_mic_driver chat.launch.py

