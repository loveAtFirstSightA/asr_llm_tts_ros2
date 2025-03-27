import sys
# TODO 加载conda环境
sys.path.append(f'/home/lio/miniconda3/envs/asr_llm_tts/lib/python3.10/site-packages')
import os
import time
from loguru import logger
import collections
import webrtcvad
import rclpy
from rclpy.node import Node
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from std_msgs.msg import String  # 用于接收输入路径和发布语音段信息

# 配置 loguru 输出到终端（不写入文件）
logger.remove()
logger.add(
    sys.stdout,
    format="[{time:YYYY-MM-DD HH:mm:ss.SSS}] [{level}] {message}",
    level="INFO"
)
logger.info('Using loguru log system')

class RealTimeVADNode(Node, FileSystemEventHandler):
    def __init__(self):
        super().__init__('rt_vad')
        # 声明参数
        self.declare_parameter('min_silence_frames', 25)
        self.min_silence_frames_ = self.get_parameter('min_silence_frames').get_parameter_value().integer_value
        self.input_path = None
        self.output_dir = None
        self._init_params()
        self._verify_directory()
        
        # 创建一个发布者，用于发布语音段信息
        self.publisher_ = self.create_publisher(String, 'pcm_file', 10)
        
        # 创建一个订阅者，用于接收输入路径
        self.subscription = self.create_subscription(String, 'rt_vad_path', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        # 初始化文件观察者
        self.observer = Observer()

    def _init_params(self):
        """初始化参数"""
        self.sample_rate = 16000       # 16kHz采样率
        self.frame_duration = 30       # 30ms帧长
        self.aggressiveness = 3        # VAD激进程度
        self.bytes_per_sample = 2      # 16bit=2字节
        self.frame_size = int(self.sample_rate * self.frame_duration / 1000)
        self.frame_bytes = self.frame_size * self.bytes_per_sample
        # self.min_silence_frames = 50 #5    # 结束静音帧数阈值
        self.min_silence_frames = self.min_silence_frames_  # 结束静音帧数阈值
        logger.info('结束静音帧数阈值: %d' % self.min_silence_frames)
        self.output_dir = os.path.abspath("/home/lio/asr_llm_tts_ros2/src/ifly_mic_driver/audio/segments")  # 默认输出目录

    def _verify_directory(self):
        """确保输出目录存在"""
        os.makedirs(self.output_dir, exist_ok=True)

    def listener_callback(self, msg):
        """接收输入路径的回调函数"""
        input_path = msg.data
        logger.info(f"收到新的输入路径: {input_path}")
        
        # 如果已经存在旧的观察者，先停止它
        if self.observer.is_alive():
            self.observer.stop()
            self.observer.join()
        
        # 更新输入路径并重新启动观察者
        self.input_path = input_path
        self._init_processing_state()
        self.observer.schedule(self, path=os.path.dirname(input_path))
        self.observer.start()
        logger.info(f"开始监控音频文件: {input_path}")

    def _init_processing_state(self):
        """初始化处理状态"""
        self.last_size = 0             # 已处理文件大小
        self.active_segment = None     # 当前活动语音段
        self.buffer = b''              # 未完成帧缓存
        self.history = collections.deque(maxlen=10)  # 语音状态历史
        self.vad = webrtcvad.Vad(self.aggressiveness)
        self.segment_counter = 0      # 语音段计数器

    def on_modified(self, event):
        """文件修改事件处理"""
        if event.src_path == self.input_path:
            try:
                self._process_new_data()
            except Exception as e:
                logger.error(f"处理数据时发生错误: {str(e)}")

    def on_created(self, event):
        """处理文件新建事件（应对文件轮转）"""
        if event.src_path == self.input_path:
            logger.info("检测到新文件，重置处理状态")
            self._init_processing_state()

    def _process_new_data(self):
        """处理新增音频数据"""
        with open(self.input_path, 'rb') as f:
            f.seek(self.last_size)
            new_data = f.read()
            self.last_size = f.tell()

        if not new_data:
            return

        # 处理数据块
        data = self.buffer + new_data
        processable = len(data) // self.frame_bytes * self.frame_bytes
        
        for offset in range(0, processable, self.frame_bytes):
            frame = data[offset:offset+self.frame_bytes]
            self._analyze_frame(frame)

        # 保存未完成数据
        self.buffer = data[processable:]

    def _analyze_frame(self, frame):
        """分析单个音频帧"""
        try:
            is_speech = self.vad.is_speech(frame, self.sample_rate)
        except:
            is_speech = False

        self.history.append(is_speech)

        # 语音开始检测
        if not self.active_segment and is_speech:
            if any(not h for h in self.history):  # 之前有静音
                self._start_new_segment(frame)

        # 语音持续处理
        elif self.active_segment:
            self.active_segment['frames'].append(frame)
            self.active_segment['duration'] += self.frame_duration

            # 静音检测
            if not is_speech:
                self.active_segment['silence_frames'] += 1
                if self.active_segment['silence_frames'] >= self.min_silence_frames:
                    self._finalize_segment()
            else:
                self.active_segment['silence_frames'] = 0

    def _start_new_segment(self, first_frame):
        """开始新的语音段"""
        self.segment_counter += 1
        self.active_segment = {
            'start_time': time.time(),
            'frames': [first_frame],
            'silence_frames': 0,
            'duration': self.frame_duration
        }
        logger.info(f"检测到语音开始 @ {time.ctime()}")

    def _finalize_segment(self):
        """完成当前语音段"""
        if self.active_segment:
            # 生成输出文件
            filename = f"segment_{self.segment_counter}_{int(time.time())}.pcm"
            output_path = os.path.join(self.output_dir, filename)
            
            try:
                with open(output_path, 'wb') as f:
                    for frame in self.active_segment['frames']:
                        f.write(frame)
                # 发布语音段信息
                msg = String()
                # msg.data = f"NEW_SEGMENT|{os.path.abspath(output_path)}|{self.active_segment['duration']}ms"
                msg.data = f"{os.path.abspath(output_path)}"
                self.publisher_.publish(msg)
                logger.info(f"生成语音段: {os.path.abspath(output_path)} "
                                       f"时长: {self.active_segment['duration']}ms")
                
            except IOError as e:
                logger.error(f"文件写入失败: {str(e)}")
            
            self.active_segment = None

    def shutdown(self):
        """关闭处理器"""
        self.observer.stop()
        self.observer.join()
        logger.info("VAD处理器已关闭")

def main(args=None):
    rclpy.init(args=args)
    processor = RealTimeVADNode()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.shutdown()
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()