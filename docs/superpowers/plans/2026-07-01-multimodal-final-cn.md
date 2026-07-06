# ESP32-P4 多模态缺陷检测最终执行计划（F:/AI-detection 版）

> 基于 `F:/stm/AI-detection/docs/superpowers/plans/2026-06-30-multimodal-one-class-svm-cn.md` 整理，并适配当前 `F:/AI-detection` 的模块化架构。

---

## 一、最终目标

做一个可演示、可答辩的多模态工业质检系统：

```text
样品进入摄像头画面
→ 系统自动检测到样品并框出
→ 视觉模型判断表面是否异常
→ 听觉 One-Class SVM 判断敲击声是否像好件
→ 融合结果显示 GOOD / BAD
```

屏幕效果：

```text
无样品：隐藏框，或显示 WAITING
检测到样品：黄色框 + CHECKING
判定好件：绿色框 + GOOD
判定坏件：红色框 + BAD
底部显示听觉分析结果
右侧显示 PASS / FAIL 计数
```

注意：样品框**框住整个样品**，不框具体伤痕。视觉模型为分类模型，只输出 GOOD/BAD。

---

## 二、系统总体方案

### 2.1 视觉模态

作用：检测样品表面可见伤痕。

流程：

```text
摄像头画面
→ sample_roi 背景差分找到样品区域
→ 裁剪样品区域
→ 按当前视觉模型规格（640×640 RGB）缩放/转换
→ ESP-DL 分类模型
→ 输出视觉 GOOD / BAD
```

复用资源：

```text
main/sample_roi.cpp/.hpp            # 已迁移的动态样品框
main/ai_vision.cpp/.h               # 现有视觉推理任务
model/best_model.espdl              # 需要重新训练替换
```

### 2.2 听觉模态

作用：判断敲击声音是否像好件。

流程：

```text
I2S 麦克风采样
→ audio_capture 读取音频
→ audio_analysis FFT 提取 8 维特征
→ audio_ocsvm One-Class SVM 推理
→ 输出听觉 GOOD / BAD
```

复用资源：

```text
main/audio_capture.cpp/.h           # I2S 采集
main/audio_analysis.cpp/.h          # FFT 与特征提取
```

新增资源：

```text
main/audio_ocsvm.cpp/.hpp           # 板上 SVM 推理
main/audio_ocsvm_params.h           # 训练好的 SVM 参数
tools/train_audio_ocsvm.py          # PC 端训练脚本
```

### 2.3 多模态融合

规则：

```text
视觉 BAD 或 听觉 BAD  → 最终 BAD
视觉 GOOD 且 听觉 GOOD → 最终 GOOD
```

条件：仅在检测到样品且听觉有效时做融合判定。

---

## 三、最终交付物

1. 训练好的视觉模型 `model/best_model.espdl`
2. 动态样品框 UI（黄/绿/红）
3. 视觉 GOOD/BAD 判断
4. 听觉 One-Class SVM 判断
5. 多模态融合 GOOD/BAD
6. 音频特征数据 `data/audio_features/good.csv`、`data/audio_features/bad_eval.csv`
7. SVM 参数 `main/audio_ocsvm_params.h`
8. 30-50 次整机测试记录
9. 45-75 秒 Demo 视频
10. PPT 与答辩话术

---

## 四、详细执行步骤

### Phase 1：数据与目录准备（Day 1）

视觉数据：

```text
dataset/
  normal/      # 200-300 张正常样品
  scratch/     # 200-300 张缺陷样品

dataset_test/
  normal/      # 25-40 张，不参与训练
  scratch/     # 25-40 张，不参与训练
```

听觉数据：

```text
data/audio_features/
  good.csv      # 200-300 次好件敲击特征
  bad_eval.csv  # 50-100 次坏件敲击特征，仅评估
```

采集要求：

```text
视觉：样品放在实际检测位、光照稳定、正常/缺陷数量平衡
听觉：敲击位置固定、麦克风距离固定、力度一致、环境安静
```

---

### Phase 2：视觉模型训练与上板（Day 2）

步骤：

1. 使用 `tools/train_defect_classifier.py` 训练分类模型。
2. 导出 `best_model.espdl`，替换 `model/best_model.espdl`。
3. 保持 `app_config.h` 中 `AI_INPUT_W = 640`、`AI_INPUT_H = 640` 和 RGB 输入。
4. 修改 `main/ai_vision.cpp`：
   - 推理触发条件增加 `g_sample_present`。
   - 预处理从动态样品框区域裁剪，再缩放到 640×640。
   - 如果时间紧，可先沿用整屏/固定视野推理，样品框仅作 UI 显示。

验证：

```text
上电后视觉能输出 NG/OK 概率
测试集准确率稳定
```

---

### Phase 3：动态样品框（Day 3）

`main/sample_roi.cpp/.hpp` 已迁移到 F 盘工程，接下来完成集成。

新增状态（放在 `app_globals.h` / `app_types.h` 或相关模块）：

```cpp
static sample_roi_t g_sample_roi = {};
static volatile bool g_sample_present = false;
```

修改 `main/video_pipeline.cpp`：

```text
在 camera_video_frame_operation 中，对 PPA 输出帧调用 sample_roi_detect_rgb565
连续 3 帧稳定后设置 g_sample_present = true
连续多帧未检测到样品则设置 g_sample_present = false
```

修改 `main/ui_manager.cpp`：

```text
新增 sample_box 和 sample_result_label
无样品：隐藏框
检测到样品：黄色框 + CHECKING
融合判定后：绿色 GOOD / 红色 BAD
```

验证：

```text
空画面：无框
放入样品：出现黄色框 CHECKING
移动样品：框能跟随
拿走样品：框隐藏
```

---

### Phase 4：听觉 One-Class SVM 训练（Day 4）

步骤：

1. 在 `main/audio_analysis.cpp` 或 `main/audio_capture.cpp` 中，当 `hit_pulse` 有效时串口输出 8 维特征：

```text
AUDIO_FEATURE,total_energy,spectral_centroid,low_energy,mid_energy,high_energy,high_mid_ratio,peak_bin,bandwidth
```

2. 采集好件/坏件敲击，整理为 `good.csv`、`bad_eval.csv`。
3. 编写 `tools/train_audio_ocsvm.py`：

```bash
python tools/train_audio_ocsvm.py \
  --good_csv data/audio_features/good.csv \
  --bad_csv data/audio_features/bad_eval.csv \
  --out main/audio_ocsvm_params.h \
  --nu 0.05 --gamma scale --max_support 80
```

4. 目标：

```text
支持向量数 <= 80
好件通过率 >= 90%
坏件拒绝率尽可能高
```

5. 若支持向量过多，调参：

```bash
--nu 0.03 --gamma 0.05 --max_support 80
```

---

### Phase 5：听觉模型上板（Day 5）

新增文件：

```text
main/audio_ocsvm.hpp
typedef struct {
    float total_energy;
    float spectral_centroid;
    float low_energy;
    float mid_energy;
    float high_energy;
    float high_mid_ratio;
    float peak_bin;
    float bandwidth;
} audio_features_t;

typedef struct {
    float score;
    bool is_pass;
} audio_ocsvm_result_t;

audio_ocsvm_result_t audio_ocsvm_predict(const audio_features_t *features);
```

```text
main/audio_ocsvm.cpp       # 推理实现：标准化 + RBF 核 + 求和
main/audio_ocsvm_params.h  # 训练脚本生成的参数
```

修改 `main/CMakeLists.txt`：

```cmake
"audio_ocsvm.cpp"
```

在 `main/audio_capture.cpp` / `audio_process_task` 中：

```text
撞击有效时，从 audio_analysis_result_t 提取 8 维特征
调用 audio_ocsvm_predict
将结果发送到 ui_event_queue / 全局变量 g_audio_svm_pass
```

验证：

```text
敲击好件：底部显示 ACOUSTIC GOOD
敲击异常件：底部显示 ACOUSTIC BAD
串口输出 SVM score
```

---

### Phase 6：多模态融合、照明与最终 UI（Day 6）

照明控制（复用现有 `main/lighting.cpp`）：

```text
CMD_VISION_START  → lighting_set_brightness(80)  // 按 START 自动开灯
CMD_VISION_STOP   → lighting_set_brightness(0)   // 停止检测关灯
CMD_BTN_EMERGENCY → lighting_set_brightness(0)   // 急停关灯
```

新增全局状态：

```cpp
static volatile bool g_vision_pass = true;
static volatile bool g_audio_svm_pass = true;
static volatile bool g_audio_svm_valid = false;
```

融合逻辑（在 UI 更新或主状态机中）：

```cpp
bool final_good = false;
if (g_sample_present && g_audio_svm_valid) {
    final_good = g_vision_pass && g_audio_svm_pass;
}
```

UI 更新：

```cpp
if (g_sample_present) {
    update_sample_box_position(&g_sample_roi);
    update_sample_box_result(final_good);
} else {
    update_sample_box_position(NULL);
}
```

显示映射：

```text
视觉 OK + 听觉 OK → 绿色框 GOOD
视觉 NG + 听觉 OK → 红色框 BAD
视觉 OK + 听觉 NG → 红色框 BAD
视觉 NG + 听觉 NG → 红色框 BAD
```

---

### Phase 7：整机测试与 Demo（Day 7）

测试 30-50 次，记录：

```text
样品编号 | 真实标签 | 是否检测到框 | 视觉结果 | 听觉结果 | 融合结果 | 是否正确 | 备注
```

Demo 视频流程（45-75 秒）：

```text
1. 系统启动，画面无样品，无框
2. 放入好件，出现黄色框 CHECKING
3. 判定后显示绿色框 GOOD
4. 拿走好件，框消失
5. 放入坏件，出现黄色框 CHECKING
6. 判定后显示红色框 BAD
7. 展示底部听觉结果与右侧计数
```

---

### Phase 8：PPT 与答辩（Day 8）

答辩话术：

```text
本系统采用视觉+听觉双模态检测。视觉模态通过摄像头自动检测样品区域，使用图像分类模型识别表面缺陷；听觉模态通过 I2S 麦克风采集敲击声，使用 FFT 提取声学特征，并采用 One-Class SVM 学习好件声纹分布。当视觉或听觉任一模态检测到异常时，系统判定为 BAD，并在屏幕样品框上显示结果。
```

常见问题：

**Q：听觉是不是机器学习？**

```text
是。FFT 负责特征提取，One-Class SVM 是听觉模态的机器学习异常检测模型。它通过好件样本学习正常声纹边界，不是简单阈值判断。
```

**Q：为什么用 One-Class SVM？**

```text
好件样本容易稳定采集，坏件类型多、难以覆盖完整。One-Class SVM 适合只学习好件分布，把不像好件的样品判为异常。
```

**Q：为什么不框住具体伤痕？**

```text
本项目视觉模型采用分类模型，目标是判断样品 GOOD/BAD，不输出伤痕坐标，因此屏幕框住的是样品检测区域。如需定位伤痕，需进一步使用目标检测或分割模型。
```

---

## 五、最终取舍

**主线只做：**

```text
动态框住样品 + 显示 GOOD/BAD
```

**不做：**

```text
自动框住具体伤痕
YOLO 伤痕检测框
图像分割
复杂 MLP 音频模型
```

**原因：**

```text
8 天内要保证能稳定演示
分类模型足够完成 GOOD/BAD 判定
动态样品框已能体现系统完整性
One-Class SVM 能证明听觉模态用了机器学习
```

---

## 六、文件变更清单

### 新增文件

```text
main/audio_ocsvm.hpp
main/audio_ocsvm.cpp
main/audio_ocsvm_params.h
main/lighting.cpp/.h             # 已存在，按 START 自动亮灯
tools/train_audio_ocsvm.py
data/audio_features/good.csv
data/audio_features/bad_eval.csv
docs/test-results-multimodal.md
docs/defense-script-multimodal.md
```

### 修改文件

```text
main/CMakeLists.txt              # 添加 audio_ocsvm.cpp
main/ai_vision.cpp               # ROI 区域裁剪输入
main/video_pipeline.cpp          # 调用 sample_roi 检测
main/ui_manager.cpp/.h           # 样品框 UI + START/STOP/急停时控制照明
main/lighting.cpp/.h             # 已存在的 GPIO4 PWM 照明
main/audio_capture.cpp           # 敲击时输出特征/调用 SVM
main/app_globals.h               # 新增全局状态
```

### 替换文件

```text
model/best_model.espdl           # 重新训练后的视觉模型
```
