# 声学异常检测模块设计文档

## 1. 背景与目标

项目使用 ESP32-P4 对金属垫片/小型刚性金属件进行多模态缺陷检测。视觉负责检测表面可见伤痕；声学负责检测摄像头无法看到的内部/背面缺陷（如背面缺损、裂纹、厚度不均）。

本文档描述**声学异常检测模块**的设计：仅学习正常零件的敲击共振特征，通过重构误差判断当前零件是否异常。

## 2. 范围与阶段

### 当前阶段（第一阶段）
- 针对**一种零件**（金属垫片类）建立声学异常检测能力。
- 模块以**可插拔组件**形式实现，可独立采集、训练、推理、验证。
- 不深度修改现有视觉 UI 流程，仅通过日志和简易状态输出异常分数。

### 后续阶段（第二阶段）
- 视觉升级为检测+分类（表面伤痕）。
- 声学模块通过统一 API 向视觉模块提供 `audio_anomaly_score`。
- 视觉与听觉分数做 late fusion，输出最终 NG/OK 决策。

## 3. 约束条件

- 硬件：ESP32-P4 + I2S 数字麦克风 + 继电器敲击头
- 采样率：16 kHz，单声道，16-bit
- 单次录音：1 秒（100 ms 预触发 + 900 ms 后触发）= 16000 样本
- 推理框架：esp-dl 3.x
- 开发语言：C/C++（设备端），Python（PC 端训练）
- 存储：SD 卡保存原始音频，PSRAM 用于中间缓冲

## 4. 整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                         设备端                               │
│  [I2S 麦克风] → [1s PCM] → [Mel 谱图提取] → [Autoencoder]   │
│                                                ↓            │
│                                         [重构 MSE]          │
│                                                ↓            │
│                              [与校准阈值比较 → NG/OK 分数]   │
└─────────────────────────────────────────────────────────────┘
                              ↑
                    采集 good 样本到 SD 卡
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                         PC 端                                │
│  [good/*.raw] → [Mel 谱图生成] → [训练 Autoencoder]          │
│                          ↓                                   │
│                [导出 audio_autoencoder.espdl]                │
└─────────────────────────────────────────────────────────────┘
```

## 5. 特征提取参数

| 参数 | 取值 | 说明 |
|------|------|------|
| 采样率 | 16 kHz | 与 I2S 配置一致 |
| 单帧 FFT | 512 | 频率分辨率 31.25 Hz |
| 帧移 (hop) | 256 | 50% 重叠 |
| 窗函数 | Hann | 与现有 FFT 一致 |
| 原始帧数 | ~61 帧/1s | 从 1s 音频得到 |
| 时间下采样 | 每 2 帧取 1 | 最终 32 帧 |
| Mel bins | 32 | 覆盖 0~8 kHz |
| 输入张量 | `[1, 32, 32, 1]` float32 | NHWC；log-Mel 谱归一化到 [0, 1]，对应 decoder 输出用 sigmoid |

## 6. 设备端模块设计

### 新增文件

| 文件 | 职责 |
|------|------|
| `main/app_audio_def.h` | 公共常量（采样率、Mel 参数、输入形状） |
| `main/app_audio_features.c/h` | Mel 谱图提取：FFT → 功率谱 → Mel 滤波 → log → 归一化 |
| `main/app_audio_ai.c/h` | 模型加载、推理、异常分数计算、校准统计、阈值管理 |

### 核心 API

```c
// 初始化模型和阈值（模型通过 CMake 嵌入）
esp_err_t audio_ai_init(const void *model_addr);

// 对一段 PCM 做异常检测
// 输出 anomaly_score：归一化后的异常分数（0 表示正常，越大越异常）
// 输出 recon_mse：原始重构误差（可选）
esp_err_t audio_ai_infer(const int16_t *pcm, uint32_t sample_count,
                         float *anomaly_score, float *recon_mse);

// 校准模式：喂入一个确认正常的样本，更新内部统计
esp_err_t audio_ai_learn_good_sample(const int16_t *pcm, uint32_t sample_count);

// 结束校准，根据已收集的正常样本计算阈值
esp_err_t audio_ai_finalize_threshold(void);

// 保存/加载阈值到 SD 卡
esp_err_t audio_ai_save_threshold(const char *path);
esp_err_t audio_ai_load_threshold(const char *path);

// 获取当前阈值参数（调试用）
esp_err_t audio_ai_get_threshold(float *mean, float *std, float *k);
```

### 异常分数计算

1. 对输入 PCM 提取 Mel 谱图 `M`（形状 `[32, 32, 1]`）。
2. Autoencoder 输出重构谱图 `M_hat`。
3. 计算重构误差：
   ```
   mse = mean((M - M_hat)^2)
   ```
4. 校准阶段收集 N 个正常样本的 `mse`，计算均值 `μ` 和标准差 `σ`。
5. 推理时：
   ```
   anomaly_score = (mse - μ) / σ
   ```
6. 判定：
   - `anomaly_score < k`：OK（k 默认 3）
   - `anomaly_score >= k`：NG

## 7. 模型架构

输入：`[1, 32, 32, 1]` float32（NHWC，与 esp-dl 一致；PC 端训练脚本需保证导出前为 NHWC）

### Encoder

```text
Conv2D(8, 3x3, padding=same) + ReLU
MaxPool2D(2x2)               → [16, 16, 8]
Conv2D(4, 3x3, padding=same) + ReLU
MaxPool2D(2x2)               → [8, 8, 4]
Flatten
Dense(16) + ReLU             → bottleneck [16]
```

### Decoder

```text
Dense(8*8*4=256) + ReLU
Reshape [8, 8, 4]
Conv2DTranspose(8, 3x3, padding=same) + ReLU   → [8, 8, 8]
UpSampling2D(2x2)                              → [16, 16, 8]
Conv2DTranspose(4, 3x3, padding=same) + ReLU   → [16, 16, 4]
UpSampling2D(2x2)                              → [32, 32, 4]
Conv2D(1, 3x3, padding=same, sigmoid/linear)   → [32, 32, 1]
```

### 模型规模预估

- 参数量：约 10K ~ 30K
- 模型文件大小：约 20KB ~ 60KB
- 单次推理时间：约 5ms ~ 20ms（ESP32-P4）

## 8. PC 端训练流程

### 8.1 数据集准备

从设备 SD 卡拷贝正常样本到 PC：

```
dataset/good/
  ├── aud0001.raw
  ├── aud0002.raw
  └── ...
```

运行数据预处理脚本：

```bash
python tools/prepare_audio_dataset.py \
  --input dataset/good \
  --output dataset/good_mels.npy
```

输出：
- `good_mels.npy`：形状 `[N, 32, 32, 1]` 的 Mel 谱图

### 8.2 训练

```bash
python tools/train_audio_autoencoder.py \
  --data dataset/good_mels.npy \
  --epochs 200 \
  --batch-size 32 \
  --latent 16 \
  --output models/audio_autoencoder
```

输出：
- `audio_autoencoder.pth`
- `threshold.json`：包含 `mean`、`std`、`k`（PC 端统计的正常样本分布）

### 8.3 导出 espdl

```bash
python tools/export_espdl.py \
  --checkpoint models/audio_autoencoder.pth \
  --output models/audio_autoencoder.espdl
```

输出：`audio_autoencoder.espdl`，复制到项目 `model/` 目录。

### 8.4 调试可视化

```bash
python tools/convert_raw_to_mel.py --input aud0001.raw --plot
```

用于检查 Mel 谱图、验证预处理是否正确。

## 9. 设备端集成点

### 9.1 数据收集

沿用现有 `audio_process_task` 保存原始 PCM，但路径改为：

```
/sdcard/audio/good/audXXXX.raw   // 正常样本，用于后续训练
/sdcard/audio/test/audXXXX.raw   // 待测样本，用于验证
```

### 9.2 CMake 嵌入模型

```cmake
target_add_binary_data(${COMPONENT_LIB} "../model/audio_autoencoder.espdl" BINARY)
```

### 9.3 main.cpp 集成（第一阶段最简版）

在 `audio_process_task` 中调用：

```c
float score = 0;
float mse = 0;
audio_ai_infer(cap.samples, cap.sample_count, &score, &mse);
ESP_LOGI(TAG, "Audio anomaly score: %.2f, mse: %.4f", score, mse);
```

暂时只通过日志输出，后续视觉模块完成后再把 `score` 接入 UI 和融合决策。

### 9.4 校准流程

1. 上电后进入校准模式（可通过按钮/编译宏/SD 卡配置文件触发）。
2. 连续敲击 30~50 个确认完好的零件。
3. 每个样本调用 `audio_ai_learn_good_sample()`。
4. 调用 `audio_ai_finalize_threshold()` 计算阈值。
5. 调用 `audio_ai_save_threshold("/sdcard/audio/threshold.json")` 保存。

正常上电时：
1. 调用 `audio_ai_load_threshold()` 加载已有阈值。

## 10. 与视觉模块的融合接口（第二阶段）

声学模块只输出一个浮点数：

```c
float audio_anomaly_score;  // 0=正常，越大越异常
```

视觉模块输出：

```c
float visual_defect_score;  // 0=正常，越大越异常（如缺陷框置信度）
```

Late fusion 示例：

```c
float fusion_score = 0.6f * visual_defect_score + 0.4f * audio_anomaly_score;
bool final_ng = fusion_score > FUSION_THRESHOLD;
```

权重和阈值根据实际产线误检/漏检率调优。

## 11. 文件改动清单

### 新增文件

- `main/app_audio_def.h`
- `main/app_audio_features.c`
- `main/app_audio_features.h`
- `main/app_audio_ai.c`
- `main/app_audio_ai.h`
- `tools/prepare_audio_dataset.py`
- `tools/train_audio_autoencoder.py`
- `tools/export_espdl.py`
- `tools/convert_raw_to_mel.py`
- `docs/superpowers/specs/2026-06-15-audio-anomaly-detection-design.md`

### 修改文件

- `main/main.cpp`：在 `audio_process_task` 中调用 `audio_ai_infer()`，增加校准模式入口（最小化改动）
- `main/CMakeLists.txt`：添加新源文件，嵌入声学模型
- `main/idf_component.yml`：确认 esp-dsp 依赖已包含

## 12. 风险评估与备选

| 风险 | 影响 | 应对 |
|------|------|------|
| 敲击力度/位置不一致导致同一件零件分数波动 | 高 | 建议改用固定行程电磁铁敲击头；装夹使用专用夹具；Mel 谱图做时间对齐和归一化 |
| 环境噪声被误判为缺陷 | 中 | 增加敲击触发检测；仅分析敲击后 200~800ms 窗口；录制环境底噪做减除 |
| 正常样本数量不足导致阈值不准 | 中 | 用数据增强（加噪、时间拉伸、幅度缩放）扩充样本 |
| Autoencoder 重构误差对微小缺陷不敏感 | 中 | 如效果不佳，后续可切换为 CNN 分类器（需收集少量缺陷样本）或变分自编码器（VAE） |
| esp-dl 对转置卷积支持有限 | 低 | 如不支持可用上采样+普通卷积替代 Decoder |

## 13. 验收标准

- [ ] 设备端能对 1s 敲击音频提取 32×32 Mel 谱图
- [ ] PC 端能训练出 < 100KB 的 Autoencoder 并导出 espdl
- [ ] 设备端加载模型后，对正常样本 anomaly_score 集中在 0~3 之间
- [ ] 对明显异常样本（如在零件上贴胶带/加重物模拟）anomaly_score 显著偏高
- [ ] 校准阈值可保存到 SD 卡并在重启后加载
- [ ] 模块 API 清晰，可被后续视觉模块无缝调用

## 14. 待确认问题

1. 是否需要在设备端实现**校准模式 UI 按钮**，还是通过 SD 卡配置文件切换？
2. 是否接受 Mel 谱图输入张量为 `[1, 32, 32, 1]`，还是更倾向 `[1, 64, 32, 1]`？
3. 阈值倍数 `k` 默认 3，是否需要可在运行时通过菜单调整？
