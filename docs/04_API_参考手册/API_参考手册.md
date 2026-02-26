# 4. API 参考手册

本章节提供 Aura Alpha 所有 ROS2 节点的完整接口定义，包括话题、参数和配置说明。

---

## 📑 目录

- [驱动层接口](#驱动层接口)
  - [audio_vm8960_node](#audio_vm8960_node)
  - [motor_node](#motor_node)
- [感知层接口](#感知层接口)
  - [mono2d_body_detection_node](#mono2d_body_detection_node)
  - [body_tracking_node](#body_tracking_node)
- [应用层接口](#应用层接口)
  - [cloud_realtime_node](#cloud_realtime_node)
  - [display_node](#display_node)
  - [easter_egg_node](#easter_egg_node)

---

## 驱动层接口

### audio_vm8960_node

WM8960 音频芯片驱动节点，提供麦克风采集和扬声器播放功能。支持三种运行模式：

| 模式 | Launch 文件 | 说明 |
|------|------------|------|
| 双工模式 | `audio_duplex.launch.py` | 同时录音和播放（推荐） |
| 录音模式 | `audio_publisher.launch.py` | 仅麦克风采集 |
| 播放模式 | `audio_player.launch.py` | 仅扬声器播放 |

#### 话题接口

**发布话题**

| 话题名称 | 消息类型 | 默认值 | 说明 |
|---------|---------|--------|------|
| `/audio_data` | `std_msgs/msg/UInt8MultiArray` | `audio_data` | 麦克风原始音频数据（PCM 格式） |

**订阅话题**

| 话题名称 | 消息类型 | 默认值 | 说明 |
|---------|---------|--------|------|
| `/audio_playback` | `std_msgs/msg/UInt8MultiArray` | `audio_playback` | 待播放的音频数据（PCM 格式） |

#### 参数定义

**基础参数**

| 参数名 | 类型 | 默认值 | 范围 | 说明 |
|--------|------|--------|------|------|
| `sample_rate` | int | `16000` | Hz | 音频采样率 |
| `channels` | int | `2` | 1-2 | 通道数（1=单声道，2=双声道） |
| `period_size` | int | `320` | 采样点 | 每个音频块的采样点数 |
| `device` | string | `plughw:0,0` | - | ALSA 设备名称 |

**话题配置**

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `input_topic` | string | `audio_data` | 录音发布话题名 |
| `output_topic` | string | `audio_playback` | 播放订阅话题名 |
| `publish_rate` | float | `50.0` | 录音发布频率（Hz） |

**录音参数**

| 参数名 | 类型 | 默认值 | 范围 | 说明 |
|--------|------|--------|------|------|
| `left_input_boost_volume` | int | `3` | 0-7 | 左声道输入增益 |
| `right_input_boost_volume` | int | `3` | 0-7 | 右声道输入增益 |
| `capture_volume` | int | `40` | 0-63 | 录音音量 |
| `adc_pcm_capture_volume` | int | `200` | 0-255 | ADC PCM 录音音量 |

**播放参数**

| 参数名 | 类型 | 默认值 | 范围 | 说明 |
|--------|------|--------|------|------|
| `speaker_dc_volume` | int | `3` | 0-5 | 扬声器 DC 音量 |
| `speaker_ac_volume` | int | `3` | 0-5 | 扬声器 AC 音量 |
| `speaker_playback_volume` | int | `127` | 0-127 | 扬声器播放音量 |
| `playback_volume` | int | `255` | 0-255 | 总播放音量 |
| `enable_headphone` | bool | `false` | - | 是否启用耳机输出 |
| `headphone_playback_volume` | int | `80` | 0-127 | 耳机播放音量 |

**队列配置**

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `record_queue_size` | int | `100` | 录音缓冲队列大小 |
| `playback_queue_size` | int | `500` | 播放缓冲队列大小 |

#### 配置文件示例

```yaml
# config/audio_duplex.yaml
audio_duplex_node:
  ros__parameters:
    # 基础配置
    sample_rate: 16000
    channels: 2
    period_size: 320
    device: "plughw:0,0"

    # 话题配置
    input_topic: "audio_data"
    output_topic: "audio_playback"
    publish_rate: 50.0

    # 录音参数
    left_input_boost_volume: 3
    right_input_boost_volume: 3
    capture_volume: 40
    adc_pcm_capture_volume: 200

    # 播放参数
    speaker_dc_volume: 3
    speaker_ac_volume: 3
    speaker_playback_volume: 127
    playback_volume: 255
```

#### 使用示例

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import numpy as np

class AudioSubscriber(Node):
    def __init__(self):
        super().__init__('audio_subscriber')
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            '/audio_data',
            self.audio_callback,
            10
        )

    def audio_callback(self, msg):
        # 将字节数据转换为 numpy 数组
        audio_data = np.frombuffer(bytes(msg.data), dtype=np.int16)
        self.get_logger().info(f'收到音频数据: {len(audio_data)} 采样点')

def main():
    rclpy.init()
    node = AudioSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
```

---

### motor_node

串口差速电机驱动节点，支持速度控制、IMU 数据读取和电池状态监控。

#### 话题接口

**订阅话题**

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 速度控制指令 |

**发布话题**

| 话题名称 | 消息类型 | 频率 | 说明 |
|---------|---------|------|------|
| `/imu/data` | `sensor_msgs/msg/Imu` | 10 Hz | 陀螺仪数据（角速度、姿态四元数） |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 1 Hz | 电池状态（电压、电量百分比） |
| `/motor/connected` | `std_msgs/msg/Bool` | 事件触发 | 串口连接状态 |

#### 参数定义

**串口配置**

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `serial_port` | string | `/dev/ttyUSB0` | 串口设备路径 |
| `baudrate` | int | `115200` | 串口波特率 |

**差速驱动参数**

| 参数名 | 类型 | 默认值 | 单位 | 说明 |
|--------|------|--------|------|------|
| `wheel_base` | float | `0.30` | m | 轴距（左右轮中心距离） |
| `max_linear_speed` | float | `0.5` | m/s | 最大线速度 |
| `max_angular_speed` | float | `2.0` | rad/s | 最大角速度 |

**安全参数**

| 参数名 | 类型 | 默认值 | 单位 | 说明 |
|--------|------|--------|------|------|
| `cmd_vel_timeout` | float | `0.5` | s | 无指令超时停车时间 |
| `reconnect_delay` | float | `3.0` | s | 串口断线重连间隔 |

**传感器参数**

| 参数名 | 类型 | 默认值 | 单位 | 说明 |
|--------|------|--------|------|------|
| `sensor_rate` | float | `10.0` | Hz | 传感器数据发布频率 |
| `invert_angular` | bool | `false` | - | 角速度方向取反 |

#### 运动学模型

差速驱动运动学公式：

```
v_left  = linear.x - angular.z × wheel_base / 2
v_right = linear.x + angular.z × wheel_base / 2
```

**速度约定**：
- `linear.x` > 0：前进
- `linear.x` < 0：后退
- `angular.z` > 0：逆时针旋转
- `angular.z` < 0：顺时针旋转

#### 配置文件示例

```yaml
# config/config.yaml
motor_node:
  ros__parameters:
    # 串口配置
    serial_port: "/dev/ttyUSB0"
    baudrate: 115200

    # 差速驱动参数
    wheel_base: 0.30
    max_linear_speed: 0.5
    max_angular_speed: 2.0

    # 安全参数
    cmd_vel_timeout: 0.5
    reconnect_delay: 3.0

    # 传感器参数
    sensor_rate: 10.0
    invert_angular: false
```

#### 使用示例

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, BatteryState

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # 速度控制发布者
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # IMU 订阅
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # 电池状态订阅
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10)

    def move_forward(self, speed=0.2):
        """前进"""
        msg = Twist()
        msg.linear.x = speed
        self.cmd_pub.publish(msg)

    def rotate(self, angular_speed=0.5):
        """原地旋转"""
        msg = Twist()
        msg.angular.z = angular_speed
        self.cmd_pub.publish(msg)

    def stop(self):
        """停止"""
        self.cmd_pub.publish(Twist())

    def imu_callback(self, msg):
        self.get_logger().info(
            f'IMU: roll={msg.orientation.x:.2f}, '
            f'pitch={msg.orientation.y:.2f}, '
            f'yaw={msg.orientation.z:.2f}'
        )

    def battery_callback(self, msg):
        self.get_logger().info(
            f'电池: {msg.voltage:.2f}V, {msg.percentage*100:.1f}%'
        )
```

---

## 感知层接口

### mono2d_body_detection_node

基于 BPU 加速的人体 2D 检测节点，使用 YOLO Pose 模型进行人体检测和关键点识别。

#### 话题接口

**订阅话题**

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/hbmem_img` | `hbm_img_msgs/msg/HbmMsg1080P` | MIPI 相机图像（共享内存） |
| `/image_raw` | `sensor_msgs/msg/Image` | USB 相机图像 |

**发布话题**

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/hobot_mono2d_body_detection` | `ai_msgs/msg/PerceptionTargets` | 人体检测结果 |

#### 参数定义

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `model_file_name` | string | `yolov8n_pose_bayese_640x640_nv12.bin` | 模型文件路径 |
| `ai_msg_pub_topic_name` | string | `/hobot_mono2d_body_detection` | 检测结果发布话题 |
| `image_width` | int | `640` | 输入图像宽度 |
| `image_height` | int | `640` | 输入图像高度 |
| `score_threshold` | float | `0.5` | 检测置信度阈值 |
| `nms_threshold` | float | `0.45` | NMS 阈值 |

#### 检测结果格式

`ai_msgs/msg/PerceptionTargets` 消息结构：

```
Header header
Target[] targets
  - string type           # "body"
  - Roi[] rois            # 检测框
    - int32 x_offset
    - int32 y_offset
    - int32 width
    - int32 height
    - float32 confidence
  - Point[] points        # 关键点（17个人体关键点）
    - float32 x
    - float32 y
    - float32 confidence
```

**人体关键点索引**：

| 索引 | 关键点 | 索引 | 关键点 |
|------|--------|------|--------|
| 0 | 鼻子 | 9 | 左手腕 |
| 1 | 左眼 | 10 | 右手腕 |
| 2 | 右眼 | 11 | 左髋 |
| 3 | 左耳 | 12 | 右髋 |
| 4 | 右耳 | 13 | 左膝 |
| 5 | 左肩 | 14 | 右膝 |
| 6 | 右肩 | 15 | 左踝 |
| 7 | 左肘 | 16 | 右踝 |
| 8 | 右肘 | | |

#### 环境变量

| 变量名 | 可选值 | 说明 |
|--------|--------|------|
| `CAM_TYPE` | `mipi` / `usb` / `fb` | 相机类型（默认 mipi） |

---

### body_tracking_node

人体跟踪节点，根据检测结果生成运动控制指令，实现人体跟随功能。

#### 话题接口

**订阅话题**

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/hobot_mono2d_body_detection` | `ai_msgs/msg/PerceptionTargets` | 人体检测结果 |
| `/display/ready` | `std_msgs/msg/Bool` | 显示就绪信号 |
| `/cloud_realtime/ready` | `std_msgs/msg/Bool` | 语音就绪信号 |

**发布话题**

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 速度控制指令（跟随模式） |
| `/follow_cmd_vel` | `geometry_msgs/msg/Twist` | 速度控制指令（禁用模式） |

#### 参数定义

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `track_serial_lost_num_thr` | int | `30` | 目标丢失帧数阈值 |
| `activate_wakeup_gesture` | int | `11` | 激活手势 ID |
| `linear_velocity` | float | `0.3` | 跟随线速度（m/s） |
| `angular_velocity` | float | `0.5` | 跟随角速度（rad/s） |
| `dead_zone` | float | `0.1` | 死区范围（避免抖动） |

#### 跟踪模式

通过 `settings.yaml` 中的 `enable_body_following` 控制：

| 设置值 | 发布话题 | 行为 |
|--------|---------|------|
| `true` | `/cmd_vel` | 机器人跟随人体移动 |
| `false` | `/follow_cmd_vel` | 仅发布指令，不控制电机 |

---

## 应用层接口

### cloud_realtime_node

云端实时语音 AI 交互节点，支持多家云服务商的语音识别和语音合成。

#### 话题接口

**订阅话题**

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/audio_data` | `std_msgs/msg/UInt8MultiArray` | 麦克风音频数据 |
| `/display/ready` | `std_msgs/msg/Bool` | 显示就绪信号 |

**发布话题**

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/audio_playback` | `std_msgs/msg/UInt8MultiArray` | TTS 播放音频 |
| `/ai/state` | `std_msgs/msg/String` | AI 状态 |
| `/ai/emotion` | `std_msgs/msg/String` | 情绪状态 |
| `/audio_msg` | `std_msgs/msg/String` | ASR 识别结果 |
| `/cloud_realtime/ready` | `std_msgs/msg/Bool` | 节点就绪信号 |

#### AI 状态值

| 状态值 | 说明 |
|--------|------|
| `idle` | 空闲状态 |
| `listening` | 正在监听 |
| `thinking` | 正在思考 |
| `speaking` | 正在说话 |
| `error` | 错误状态 |

#### 情绪状态值

| 状态值 | 说明 |
|--------|------|
| `neutral` | 中性 |
| `happy` | 开心 |
| `sad` | 悲伤 |
| `angry` | 生气 |
| `surprised` | 惊讶 |

#### 参数定义

**基础参数**

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `provider` | string | `volcano` | AI 服务提供商 |
| `sample_rate` | int | `16000` | 音频采样率 |
| `channels` | int | `1` | 音频通道数 |

**支持的 AI 提供商**

| 提供商 | provider 值 | 说明 |
|--------|-------------|------|
| 火山引擎 | `volcano` | 字节跳动云服务（推荐） |
| 百度 | `baidu` | 百度智能云 |
| OpenAI | `openai` | OpenAI Realtime API |
| Gemini | `gemini` | Google Gemini |

#### 配置文件示例

```yaml
# config/config.yaml
cloud_realtime_node:
  ros__parameters:
    provider: "volcano"
    sample_rate: 16000
    channels: 1

    # 火山引擎配置
    volcano:
      app_id: "your_app_id"
      access_token: "your_access_token"
      cluster: "volcano_tts"
      voice_type: "zh_female_cancan"

    # 百度配置
    baidu:
      app_id: "your_app_id"
      api_key: "your_api_key"
      secret_key: "your_secret_key"

    # OpenAI 配置
    openai:
      api_key: "your_api_key"
      model: "gpt-4o-realtime-preview"

    # Gemini 配置
    gemini:
      api_key: "your_api_key"
```

#### 提示词配置

```yaml
# config/prompts.yaml
system_prompt: |
  你是 Aura，一个友好的球形机器人助手。
  你的性格活泼开朗，喜欢和人类交流。
  请用简洁、自然的语言回答问题。

wake_word: "你好小球"
```

#### 使用示例

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AIStateMonitor(Node):
    def __init__(self):
        super().__init__('ai_state_monitor')

        self.state_sub = self.create_subscription(
            String, '/ai/state', self.state_callback, 10)

        self.emotion_sub = self.create_subscription(
            String, '/ai/emotion', self.emotion_callback, 10)

        self.asr_sub = self.create_subscription(
            String, '/audio_msg', self.asr_callback, 10)

    def state_callback(self, msg):
        self.get_logger().info(f'AI 状态: {msg.data}')

    def emotion_callback(self, msg):
        self.get_logger().info(f'情绪: {msg.data}')

    def asr_callback(self, msg):
        self.get_logger().info(f'识别结果: {msg.data}')
```

---

### display_node

屏幕显示控制节点，根据 AI 状态切换不同的动画视频。

#### 话题接口

**订阅话题**

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/ai/state` | `std_msgs/msg/String` | AI 状态 |
| `/ai/emotion` | `std_msgs/msg/String` | 情绪状态 |
| `/audio_msg` | `std_msgs/msg/String` | ASR 识别结果（用于字幕显示） |

**发布话题**

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/display/ready` | `std_msgs/msg/Bool` | 显示就绪信号 |

#### 参数定义

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `video_dir` | string | `videos/` | 视频文件目录 |
| `default_video` | string | `idle.mp4` | 默认视频 |
| `backend` | string | `gstreamer` | 渲染后端 |
| `fullscreen` | bool | `true` | 是否全屏 |
| `fps` | int | `30` | 目标帧率 |

**渲染后端选项**

| 后端 | 说明 |
|------|------|
| `gstreamer` | GStreamer 硬件加速（推荐） |
| `pygame` | Pygame 软件渲染 |
| `tkinter` | Tkinter 软件渲染 |

#### 视频映射

| AI 状态 | 视频文件 |
|---------|---------|
| `idle` | `idle.mp4` |
| `listening` | `listening.mp4` |
| `thinking` | `thinking.mp4` |
| `speaking` | `speaking.mp4` |
| `error` | `error.mp4` |

#### 配置文件示例

```yaml
# config/display.yaml
display_node:
  ros__parameters:
    video_dir: "/path/to/videos/"
    default_video: "idle.mp4"
    backend: "gstreamer"
    fullscreen: true
    fps: 30

    # 话题配置
    state_topic: "/ai/state"
    emotion_topic: "/ai/emotion"
    asr_topic: "/audio_msg"
```

---

### easter_egg_node

彩蛋功能节点，支持手势挑战和关键词触发特殊交互。

#### 话题接口

**订阅话题**

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/hobot_hand_gesture_detection` | `ai_msgs/msg/PerceptionTargets` | 手势识别结果 |
| `/audio_msg` | `std_msgs/msg/String` | ASR 识别结果 |

#### 参数定义

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `gesture_challenge_enabled` | bool | `true` | 是否启用手势挑战 |
| `keyword_trigger_enabled` | bool | `true` | 是否启用关键词触发 |

#### 手势 ID 映射

| 手势 ID | 手势名称 |
|---------|---------|
| 0 | 拳头 |
| 1 | 手掌 |
| 2 | OK |
| 3 | 竖大拇指 |
| 4 | 比心 |
| 5 | 剪刀手 |
| 11 | 唤醒手势 |

#### 配置文件示例

```yaml
# config/config.yaml
easter_egg_node:
  ros__parameters:
    gesture_challenge_enabled: true
    keyword_trigger_enabled: true

    # 关键词配置
    keywords:
      - trigger: "跳个舞"
        action: "dance"
      - trigger: "转个圈"
        action: "spin"
```

---

## 📋 消息类型速查

### 标准消息

| 消息类型 | 包名 | 说明 |
|---------|------|------|
| `UInt8MultiArray` | `std_msgs` | 字节数组（音频数据） |
| `String` | `std_msgs` | 字符串 |
| `Bool` | `std_msgs` | 布尔值 |
| `Twist` | `geometry_msgs` | 速度指令（线速度+角速度） |
| `Imu` | `sensor_msgs` | IMU 数据 |
| `BatteryState` | `sensor_msgs` | 电池状态 |
| `Image` | `sensor_msgs` | 图像数据 |

### 自定义消息

| 消息类型 | 包名 | 说明 |
|---------|------|------|
| `PerceptionTargets` | `ai_msgs` | 感知检测结果 |
| `HbmMsg1080P` | `hbm_img_msgs` | 共享内存图像 |

---

## 🔧 调试工具

### 话题监控

```bash
# 查看所有话题
ros2 topic list

# 监控特定话题
ros2 topic echo /ai/state
ros2 topic echo /cmd_vel
ros2 topic echo /battery_state

# 查看话题频率
ros2 topic hz /audio_data
ros2 topic hz /imu/data
```

### 参数查看

```bash
# 查看节点参数
ros2 param list /motor_node
ros2 param get /motor_node wheel_base

# 动态修改参数
ros2 param set /motor_node max_linear_speed 0.3
```

### 节点信息

```bash
# 查看节点列表
ros2 node list

# 查看节点详情
ros2 node info /motor_node
ros2 node info /cloud_realtime_node
```
