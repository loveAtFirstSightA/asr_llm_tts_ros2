import sys
import os
from loguru import logger

logger.info(f"当前 Python 解释器路径: {sys.executable}")
# 添加 lerobot 源码路径到 Python 路径
lerobot_path = os.path.expanduser("~/lerobot_voice")
if lerobot_path not in sys.path:
    sys.path.append(lerobot_path)
    
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from lerobot.common.policies.factory import make_policy
from lerobot.common.robot_devices.robots.utils import Robot, make_robot_from_config
from lerobot.common.robot_devices.control_utils import (
    control_loop,
    init_keyboard_listener,
    log_control_info,
    record_episode,
    reset_environment,
    sanity_check_dataset_name,
    sanity_check_dataset_robot_compatibility,
    stop_recording,
    warmup_record,
)
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.configs import parser
import time

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
        # self.declare_parameter('control_type', 'voice')
        # self.control_type_ = self.get_parameter('control_type').get_parameter_value().string_value
        # logger.info('control_type: %s' % self.control_type_)
        self.robot = None
        self.policy = None
        self.dataset = None

    def voice_command_subscriber_callback(self, msg):
        logger.info('Received message: %s' % msg.data)
        if msg.data == 'stack_towel':
            try:
                self.execute_arm_action()
            except Exception as e:
                logger.error(f"Error executing arm action: {e}")
        elif msg.data == 'start_recording':
            try:
                self.start_recording()
            except Exception as e:
                logger.error(f"Error starting recording: {e}")
        else:
            logger.warning('不支持此动作 %s' % msg.data)

    def execute_arm_action(self):
        logger.info('准备执行动作指令（当前仅支持折叠毛巾）')
        return
        # 添加控制指令
        if self.robot is None:
            self.robot = make_robot_from_config(self.get_config())
        if not self.robot.is_connected:
            self.robot.connect()
        # 这里可以添加具体的动作控制逻辑，比如发送动作指令到机器人等
        # 例如：
        # self.robot.send_action(some_action)

    def start_recording(self):
        logger.info('开始录制数据集')
        config = self.get_config()
        self.robot = make_robot_from_config(config.robot)
        if not self.robot.is_connected:
            self.robot.connect()
        self.dataset = LeRobotDataset.create(
            config.control.repo_id,
            config.control.fps,
            root=config.control.root,
            robot=self.robot,
            use_videos=config.control.video,
            image_writer_processes=config.control.num_image_writer_processes,
            image_writer_threads=config.control.num_image_writer_threads_per_camera * len(self.robot.cameras),
        )
        self.policy = make_policy(config.control.policy, ds_meta=self.dataset.meta)
        listener, events = init_keyboard_listener()
        log_say("Warmup record", config.control.play_sounds)
        warmup_record(self.robot, events, True, config.control.warmup_time_s, config.control.display_cameras, config.control.fps)
        recorded_episodes = 0
        while recorded_episodes < config.control.num_episodes:
            log_say(f"Recording episode {self.dataset.num_episodes}", config.control.play_sounds)
            record_episode(
                robot=self.robot,
                dataset=self.dataset,
                events=events,
                episode_time_s=config.control.episode_time_s,
                display_cameras=config.control.display_cameras,
                policy=self.policy,
                fps=config.control.fps,
                single_task=config.control.single_task,
            )
            if not events["stop_recording"] and (
                (recorded_episodes < config.control.num_episodes - 1) or events["rerecord_episode"]
            ):
                log_say("Reset the environment", config.control.play_sounds)
                reset_environment(self.robot, events, config.control.reset_time_s, config.control.fps)
            if events["rerecord_episode"]:
                log_say("Re-record episode", config.control.play_sounds)
                events["rerecord_episode"] = False
                events["exit_early"] = False
                self.dataset.clear_episode_buffer()
                continue
            self.dataset.save_episode()
            recorded_episodes += 1
            if events["stop_recording"]:
                break
        log_say("Stop recording", config.control.play_sounds, blocking=True)
        stop_recording(self.robot, listener, config.control.display_cameras)
        if config.control.push_to_hub:
            self.dataset.push_to_hub(tags=config.control.tags, private=config.control.private)
        log_say("Exiting", config.control.play_sounds)
        self.robot.disconnect()

    def get_config(self):
        # 这里需要根据你的配置文件解析逻辑来实现
        # 假设你已经有一个解析配置的函数，比如parser.parse_args()
        # config = parser.parse_args()
        # return config
        # 这里只是一个示例，实际实现需要根据你的项目结构调整
        import argparse
        parser = argparse.ArgumentParser()
        parser.add_argument('--robot.type', dest='robot.type', type=str, required=True)
        parser.add_argument('--control.type', dest='control.type', type=str, required=True)
        parser.add_argument('--control.fps', dest='control.fps', type=int, required=True)
        parser.add_argument('--control.single_task', dest='control.single_task', type=str, required=True)
        parser.add_argument('--control.repo_id', dest='control.repo_id', type=str, required=True)
        parser.add_argument('--control.tags', dest='control.tags', type=str, required=True)
        parser.add_argument('--control.warmup_time_s', dest='control.warmup_time_s', type=float, required=True)
        parser.add_argument('--control.episode_time_s', dest='control.episode_time_s', type=float, required=True)
        parser.add_argument('--control.reset_time_s', dest='control.reset_time_s', type=float, required=True)
        parser.add_argument('--control.num_episodes', dest='control.num_episodes', type=int, required=True)
        parser.add_argument('--control.push_to_hub', dest='control.push_to_hub', type=bool, required=True)
        parser.add_argument('--control.policy.path', dest='control.policy.path', type=str, required=True)
        args, unknown = parser.parse_known_args()
        return args

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