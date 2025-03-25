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
logger.info("正在使用loguru日志系统")

sys.path.append('/home/lio/lerobot')
# import submodules
from lerobot.scripts.control_robot import print_msg
from lerobot.scripts.control_robot import control_robot
from lerobot.common.policies.factory import make_policy
from lerobot.common.robot_devices.control_configs import ControlPipelineConfig, RecordControlConfig

class LerobotRos2(Node):

    def __init__(self):
        super().__init__('lerobot_ros2')
        self.voice_command_subscriber_ = self.create_subscription(String, 'voice_command', self.voice_command_subscriber_callback, 10)
        # define parameters
        self.declare_parameter('voice_command_list', ['12', '34'])
        self.voice_command_list_ = self.get_parameter('voice_command_list').get_parameter_value().string_array_value
        self.get_logger().info(f"Voice command list: {self.voice_command_list_}")

    def voice_command_subscriber_callback(self, msg):
        logger.info("Received voice_command: %s" % msg.data)
        print_msg("hello, lerobot")
        voice_command = msg.data
        if voice_command in self.voice_command_list_:
            if voice_command == 'stack_towel':
                self.utils_stack_towel()
            else:
                logger.info(f"Executing command: {voice_command}")
        else:
            logger.warning(f"Received voice command '{voice_command}' is not in the supported list: {self.voice_command_list_}")

    def utils_stack_towel(self):
        logger.info("Executing stack towel")
        # TODO use lerobot control_robot interface
        # control_robot()
        try:
            # 配置 ControlPipelineConfig 参数
            cfg = ControlPipelineConfig(
                robot_type='so100',
                control_type='record',
                fps=30,
                single_task="Grasp a lego block and put it in the bin.",
                repo_id='./eval_actm',
                tags=['tutorial'],
                warmup_time_s=5,
                episode_time_s=20,
                reset_time_s=3,
                num_episodes=1,
                push_to_hub=False,
                policy_path='outputs/train/2025-03-18/11-28-20_act/checkpoints/040000/pretrained_model'
            )

            # 调用 control_robot 函数
            control_robot(cfg)

        except Exception as e:
            logger.error(f"An error occurred: {e}")        

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LerobotRos2())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
