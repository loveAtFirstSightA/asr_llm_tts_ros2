#!/home/lio/miniconda3/envs/lerobot/bin/python
# -*- coding:utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from loguru import logger

logger.remove()
logger.add(
    sys.stdout,
    format="[{time:YYYY-MM-DD HH:mm:ss.SSS}] [{level}] {message}",
    level="INFO"
)
logger.info("Using loguru log system")

sys.path.append('/home/lio/lerobot')
# import submodules
from lerobot.scripts.control_robot import control_robot


class LerobotRos2(Node):

    def __init__(self):
        super().__init__('lerobot_ros2')
        self.voice_command_subscriber_ = self.create_subscription(String, 'voice_command', self.voice_command_subscriber_callback, 10)
        self.declare_parameter('voice_command_list', ['12', '34'])
        self.voice_command_list_ = self.get_parameter('voice_command_list').get_parameter_value().string_array_value
        self.get_logger().info(f"Voice command list: {self.voice_command_list_}")

    def voice_command_subscriber_callback(self, msg):
        logger.info("Received voice_command: %s" % msg.data)
        voice_command = msg.data
        if voice_command in self.voice_command_list_:
            if voice_command == 'stack_towel':
                self.utils_stack_towel()
            if voice_command == 'calibrate':
                self.utils_calibrate()
        else:
            logger.warning(f"Received voice command '{voice_command}' is not in the supported list: {self.voice_command_list_}")
    
    def utils_stack_towel(self):
        """
        python lerobot/scripts/control_robot.py \
            --robot.type=so100 \
            --control.type=record \
            --control.fps=30 \
            --control.single_task="action" \
            --control.repo_id=./eval_actm \
            --control.tags='["tutorial"]' \
            --control.warmup_time_s=5 \
            --control.episode_time_s=20 \
            --control.reset_time_s=3 \
            --control.num_episodes=1 \
            --control.push_to_hub=false \
            --control.policy.path=/home/lio/lerobot/outputs/train/act_3camera_cloth_20250317_bs10_010000_pretrained_model
        """
        logger.info("Executing stack towel")
        # TODO use lerobot interface
        original_argv = sys.argv.copy()
        sys.argv = [
            "lerobot/scripts/control_robot.py",
            "--robot.type=so100",
            "--control.type=record",
            "--control.fps=30",
            '--control.single_task="action"',
            "--control.repo_id=./eval_actm",
            '--control.tags=["tutorial"]',
            "--control.warmup_time_s=5",
            "--control.episode_time_s=20",
            "--control.reset_time_s=3",
            "--control.num_episodes=1",
            "--control.push_to_hub=false",
            "--control.policy.path=/home/lio/lerobot/outputs/train/act_3camera_cloth_20250317_bs10_010000_pretrained_model"
        ]
        control_robot()
        sys.argv = original_argv 


    def utils_calibrate(self):
        """
        python lerobot/scripts/control_robot.py \
            --robot.type=so100 \
            --control.type=calibrate
        """
        logger.info("Executing calibrate")
        # TODO use lerobot interface
        logger.info("Executing calibrate")
        original_argv = sys.argv.copy()
        sys.argv = [
            "lerobot/scripts/control_robot.py",
            "--robot.type=so100",
            "--control.type=calibrate"
        ]
        control_robot()
        sys.argv = original_argv


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LerobotRos2())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
