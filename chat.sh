#!/bin/bash

rm /home/lio/asr_llm_tts_ros2/src/ifly_mic_driver/audio/*.pcm

rm -r /home/lio/asr_llm_tts_ros2/src/ifly_mic_driver/audio/segments

colcon build && source install/local_setup.bash && ros2 launch ifly_mic_driver chat.launch.py

