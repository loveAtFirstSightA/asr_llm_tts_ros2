#!/bin/bash

rm /home/lio/asr_llm_tts_ros2/src/ifly_mic_driver/audio/*.pcm
echo '删除历史语音文件'

rm -r /home/lio/asr_llm_tts_ros2/src/ifly_mic_driver/audio/segments
echo '删除历史分段语音文件'

rm -rf build/ install/ log/
echo '删除ROS2历史编译文件'

colcon build && source install/local_setup.bash && ros2 launch ifly_mic_driver chat.launch.py
