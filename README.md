# 基于 ESP32-P4 的边缘 AI 工业质检终端

[![ESP Launchpad](https://espressif.github.io/esp-launchpad/assets/try_with_launchpad.png)](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_camera_new-)

## 项目简介

本项目是一款面向工业场景的边缘 AI 质检终端，基于 **ESP32-P4 Function EV Board** 开发。系统融合 **机器视觉** 与 **声学检测** 两种模态，可在本地实时完成零部件外观缺陷识别与敲击材质/内部异常判断，适用于小批量产线、实验室抽检、教学演示等场景。

核心创新点：
- **端侧 AI 推理**：使用 `esp-dl` 在 ESP32-P4 上直接运行 YOLO 分类模型，无需联网。
- **视觉 + 声学双模态**：外观划痕检测结合敲击声纹频谱分析，提高缺陷检出率。
- **完整闭环**：采集 → 推理 → 执行机构反馈 → 显示 → SD 卡数据记录，全流程本地完成。
- **工业风格 UI**：基于 LVGL 的实时状态面板、良率统计、声纹波形与报警卡片。

## 系统架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                        ESP32-P4 边缘 AI 质检终端                      │
├─────────────────────────────────────────────────────────────────────┤
│  摄像头 (MIPI-CSI/DVP)                                               │
│       │                                                              │
│       ▼                                                              │
│  ┌─────────────┐    ┌─────────────┐    ┌───────────────────────┐    │
│  │  PPA 缩放    │───▶│  显示缓冲    │───▶│  LVGL 工业面板 (LCD)   │    │
│  └─────────────┘    └─────────────┘    └───────────────────────┘    │
│       │                                                              │
│       ▼                                                              │
│  ┌─────────────────────┐                                             │
│  │  YOLO 视觉 AI 模型   │─── NG/OK 概率 ──▶ 报警 / 良率统计 / 执行机构 │
│  └─────────────────────┘                                             │
│                                                                      │
│  I2S 麦克风 ◀── 敲击机构（继电器）                                    │
│       │                                                              │
│       ▼                                                              │
│  ┌─────────────────────┐    ┌───────────────────────┐               │
│  │  FFT + 频谱质心分析  │───▶│  声纹波形 / 材质判定   │               │
│  └─────────────────────┘    └───────────────────────┘               │
│                                                                      │
│  SD 卡：保存缺陷截图 / 原始音频 PCM，用于离线分析与模型迭代            │
└─────────────────────────────────────────────────────────────────────┘
```

## 功能特性

| 模块 | 功能说明 |
|------|----------|
| 视觉检测 | 摄像头实时采集 → PPA 硬件缩放 → YOLO 模型推理 → 识别 normal / scratch（划痕） |
| 声学检测 | I2S 麦克风监听 → 敲击触发 → FFT 频谱分析 → 频谱质心材质判定 |
| 执行机构 | GPIO 继电器驱动敲击装置，NG 时触发剔除/报警逻辑 |
| 人机交互 | 1024×600 LCD + LVGL：状态栏、视频窗口、声纹波形、AI 结果、良率统计 |
| 数据记录 | SD 卡保存 RAW 截图与 16kHz PCM 音频，支持离线训练与问题追溯 |
| 紧急停止 | 一键 E-STOP，停止所有检测并锁定界面 |

## 硬件需求

- **主控板**：ESP32-P4-Function-EV-Board（或其他支持 MIPI-CSI/DVP 的 ESP32-S3/P4 开发板）
- **摄像头**：
  - ESP32-P4-Function-EV-Board：SC2336（正向排线）或 OV5647（反向排线）
  - ESP32-S3-EYE：OV2640
  - ESP32-P4-EYE：OV2710
- **音频**：I2S 数字麦克风（SCK: GPIO21, WS: GPIO22, SD: GPIO20）
- **执行机构**：继电器模块（GPIO23），用于驱动敲击/剔除装置
- **存储**：MicroSD 卡（用于保存图片与音频）
- **显示**：板载 LCD（1024×600，RGB565）

> 默认配置针对 ESP32-P4-Function-EV-Board。使用其他板子时，请修改 `sdkconfig.bsp.{board}` 与 `main/app_config.h` 中的引脚定义。

## 软件依赖

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf) ≥ 5.3
- 以下 ESP-IDF 组件/管理组件：
  - `espressif/esp-dl` (^3.0.0)
  - `espressif/esp32-camera`
  - `espressif/esp-dsp`
  - `espressif/esp_video`
  - `fatfs`、`sdmmc`（SDK 内置）

## 项目结构

```
.
├── main/
│   ├── main.cpp              # 系统初始化入口
│   ├── app_video.c/h         # V4L2 摄像头驱动封装
│   ├── ui_manager.cpp/h      # LVGL 界面创建与状态更新
│   ├── ai_vision.cpp/h       # YOLO 视觉 AI 推理
│   ├── audio_inspection.cpp/h # I2S 麦克风、FFT、敲击与音频保存
│   ├── hardware_control.cpp/h # 继电器/剔除机构控制
│   ├── sd_storage.cpp/h      # SD 卡图片/音频写入
│   ├── video_pipeline.cpp/h  # PPA 缩放、显示刷新、摄像头帧回调
│   ├── app_config.h          # 硬件/算法/UI 常量
│   ├── app_types.h           # 枚举与消息结构体
│   └── app_globals.h         # 跨模块全局变量声明
├── model/
│   └── best_model.espdl      # 视觉检测模型（二分类：normal / scratch）
├── convert_raw_to_png.py     # 将 SD 卡保存的 RAW 图像转换为 PNG
├── sdkconfig.bsp.*           # 各开发板默认 SDK 配置
├── partitions.csv            # 分区表
└── README.md
```

## 构建与烧录

### 1. 设置 ESP-IDF 环境

```bash
# 根据你的安装路径调整
. $HOME/esp/esp-idf/export.sh
```

### 2. 选择开发板并构建

以 ESP32-P4-Function-EV-Board 为例：

```bash
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.esp32_p4_function_ev_board build
```

其他开发板：

```bash
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.esp32_s3_eye build
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.esp32_p4_eye build
```

### 3. 烧录与监控

```bash
idf.py -p PORT flash monitor
```

将 `PORT` 替换为实际串口号，例如 Windows 下 `COM5`，Linux 下 `/dev/ttyUSB0`。

## 使用说明

1. **上电启动**：系统初始化完成后，LCD 显示工业检测主界面，状态栏显示 `SYSTEM NORMAL`。
2. **启动视觉检测**：点击右侧 **VISION START**，顶部状态变为 `VISION: RUNNING`，视频区显示 `AI: DETECTING`。
3. **启动声学检测**：将被测物放置到位，点击 **AUDIO START**，继电器自动敲击一次，系统捕获 1 秒音频并分析声纹。
4. **查看结果**：
   - 右下角显示 `AI NG / AI OK` 概率条。
   - 底部声纹区显示实时 Health Score 波形与 `SIGNAL OK / SIGNAL NG` 判定。
   - 顶部状态栏实时更新 PASS / FAIL / YIELD 统计。
5. **抓拍保存**：点击 **CAPTURE**，当前帧以 RAW 格式保存到 SD 卡 `/sdcard/images/`。
6. **紧急停止**：点击 **EMERGENCY STOP**，系统进入急停状态并锁定所有按钮。

## 性能指标

> 以下数据需在真实硬件环境下测试并填入。

| 指标 | 数值 | 备注 |
|------|------|------|
| 视觉推理输入分辨率 | 640 × 640 | RGB565 → RGB888/BGR |
| 视觉推理帧率 | ___ FPS | 默认约 5 FPS（200ms/次） |
| 预处理耗时 | ___ μs | 含 LUT 缩放与颜色空间转换 |
| 模型推理耗时 | ___ μs | 含 YOLO 前向传播 |
| 视觉检测准确率 | ___ % | 测试集：___ 张 |
| 划痕缺陷召回率 | ___ % | 需根据实际数据集填写 |
| 声学采样率 | 16 kHz | 16-bit PCM |
| FFT 帧长 | 1024 点 | 约 64ms |
| 音频截取长度 | 1000 ms | 含 100ms 预触发 |
| 声纹判定阈值 | 50 ~ 320 | 频谱质心 Health Score |

## 模型训练与替换

当前模型 `model/best_model.espdl` 为二分类模型，输出：
- `out_ptr[0]`：normal（正常）
- `out_ptr[1]`：scratch（划痕）

如需替换为自己的模型：
1. 使用 `esp-ppq` 或 `esp-dl` 工具链将 PyTorch/TensorFlow 模型转换为 `.espdl` 格式。
2. 将新模型覆盖 `model/best_model.espdl`。
3. 若输出类别或形状改变，需同步修改 `main/ai_vision.cpp` 中的后处理逻辑。

## 数据导出与可视化

SD 卡保存的图像为 RAW RGB565 格式，可使用项目根目录的 `convert_raw_to_png.py` 转换为 PNG：

```bash
python convert_raw_to_png.py
```

> 请根据实际分辨率修改脚本中的 `WIDTH` 与 `HEIGHT`。

保存的音频为 16-bit 单声道 PCM（16 kHz），可用 Audacity 等工具以 `Signed 16-bit PCM, Little-endian, 单声道, 16000 Hz` 格式导入播放与分析。

## 后续优化方向

- [ ] 引入轻量级声学 AI 分类模型，替代基于阈值的频谱质心判定。
- [ ] 扩展缺陷类别：凹陷、裂纹、色差等。
- [ ] 增加 Wi-Fi/蓝牙，将 NG 图片上传至上位机或手机 App。
- [ ] 模型 INT8 量化，进一步降低推理延迟与内存占用。
- [ ] 增加机械定位夹具，提升敲击一致性。

## 许可证

本示例代码基于 CC0-1.0 许可证发布。模型与第三方组件遵循各自原始许可证。

## 演示

> 可在此处插入项目演示视频链接或截图。
>
> - 视频链接：___
> - 截图 1：___
> - 截图 2：___
