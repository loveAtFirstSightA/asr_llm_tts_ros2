#!/home/lio/miniconda3/envs/lerobot/bin/python
# -*- coding:utf-8 -*-

import sys
import subprocess
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
        # TODO use shell lerobot control_robot interface
        # 执行 shell 脚本
        try:
            # 使用 subprocess.run 来执行 shell 脚本
            result = subprocess.run(['/home/kc/lerobot_voice/demo.sh'], check=True, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            logger.info('脚本执行成功: %s', result.stdout.decode())
        except subprocess.CalledProcessError as e:
            logger.error('脚本执行失败: %s', e.stderr.decode())
        except FileNotFoundError:
            logger.error('脚本文件未找到: /home/kc/lerobot_voice/demo.sh')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LerobotRos2())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
