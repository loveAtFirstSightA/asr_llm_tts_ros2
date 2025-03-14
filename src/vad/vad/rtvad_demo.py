import os
import time
import logging
import collections
import webrtcvad
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class RealTimeVADProcessor(FileSystemEventHandler):
    def __init__(self, input_path, output_dir):
        self.input_path = input_path
        self.output_dir = os.path.abspath(output_dir)
        self._init_audio_params()
        self._init_processing_state()
        self._verify_directory()
        
        # 启动文件观察
        self.observer = Observer()
        self.observer.schedule(self, path=os.path.dirname(input_path))
        self.observer.start()
        logging.info(f"开始监控音频文件: {input_path}")

    def _init_audio_params(self):
        """初始化音频处理参数"""
        self.sample_rate = 16000       # 16kHz采样率
        self.frame_duration = 30       # 30ms帧长
        self.aggressiveness = 3        # VAD激进程度
        self.bytes_per_sample = 2      # 16bit=2字节
        self.frame_size = int(self.sample_rate * self.frame_duration / 1000)
        self.frame_bytes = self.frame_size * self.bytes_per_sample
        self.min_silence_frames = 5    # 结束静音帧数阈值

    def _init_processing_state(self):
        """初始化处理状态"""
        self.last_size = 0             # 已处理文件大小
        self.active_segment = None     # 当前活动语音段
        self.buffer = b''              # 未完成帧缓存
        self.history = collections.deque(maxlen=10)  # 语音状态历史
        self.vad = webrtcvad.Vad(self.aggressiveness)
        self.segment_counter = 0      # 语音段计数器

    def _verify_directory(self):
        """确保输出目录存在"""
        os.makedirs(self.output_dir, exist_ok=True)

    def on_modified(self, event):
        """文件修改事件处理"""
        if event.src_path == self.input_path:
            try:
                self._process_new_data()
            except Exception as e:
                logging.error(f"处理数据时发生错误: {str(e)}")

    def on_created(self, event):
        """处理文件新建事件（应对文件轮转）"""
        if event.src_path == self.input_path:
            logging.info("检测到新文件，重置处理状态")
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
        logging.info(f"检测到语音开始 @ {time.ctime()}")

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
                
                # 发送绝对路径（示例输出）
                abs_path = os.path.abspath(output_path)
                print(f"NEW_SEGMENT|{abs_path}|{self.active_segment['duration']}ms")
                
                logging.info(f"生成语音段: {abs_path} "
                             f"时长: {self.active_segment['duration']}ms")
                
            except IOError as e:
                logging.error(f"文件写入失败: {str(e)}")
            
            self.active_segment = None

    def shutdown(self):
        """关闭处理器"""
        self.observer.stop()
        self.observer.join()
        logging.info("VAD处理器已关闭")

if __name__ == "__main__":
    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    # 使用示例
    processor = RealTimeVADProcessor(
        input_path="/tmp/audio_stream.pcm",
        output_dir="./segments"
    )

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        processor.shutdown()