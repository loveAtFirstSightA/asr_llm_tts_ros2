import sys
from loguru import logger
import subprocess
logger.info(f"当前 Python 解释器路径: {sys.executable}")
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 配置 loguru 日志
logger.remove()
logger.add(
    sys.stdout,
    format="[{time:YYYY-MM-DD HH:mm:ss.SSS}] [{level}] {message}",
    level="INFO"
)
logger.info('正在使用 loguru 日志系统')

class ControlInterface(Node):
    def __init__(self):
        super().__init__('control_interface')
        self.voice_command_subscriber_ = self.create_subscription(
            String, 'voice_command', self.voice_command_subscriber_callback, 10)
        self.declare_parameter('control_type', 'voice')
        self.control_type_ = self.get_parameter('control_type').get_parameter_value().string_value
        logger.info('control_type: %s' % self.control_type_)

    def voice_command_subscriber_callback(self, msg):
        logger.info('Received message: %s' % msg.data)
        if (msg.data == 'stack_towel'):
            try:
                self.execute_arm_action()
            except Exception as e:
                logger.error(f"Error executing arm action: {e}")
        else:
            logger.warning('不支持此动作 %s' % msg.data)

    def execute_arm_action(self):
        logger.info('准备执行动作指令（当前仅支持折叠毛巾）')
        # TODO添加控制指令
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
    node = ControlInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt, shutting down node")
    except Exception as e:
        logger.error(f"Error in main: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
