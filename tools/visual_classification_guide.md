# ESP32-P4 视觉二分类方案参考文档

> 可直接转发给负责模型训练和量化的同学。

## 一、为什么要从 YOLO 检测改成二分类？

当前 YOLO 检测模型的问题：
- 训练时标注框的是**整个样品**，模型没有学到"划痕"特征
- 检测框会飘到空白处、背景处
- 对正常样品也有响应，分类不稳定

改成**整图二分类**后：
- 输入整张图，输出 `normal` / `scratch` 两个分数
- 不需要画框，只看整图好坏
- 标注简单：每张图打一个标签即可
- 模型更小更快，和音频 AI 思路一致

---

## 二、方案规格

| 项目 | 规格 |
|------|------|
| 模型架构 | MobileNetV3-Small |
| 输入尺寸 | 320 × 320 |
| 颜色空间 | RGB |
| 预处理 | letterbox + 灰边 114 + `/255` 归一化 |
| 输出 | `[1, 2]` logits：`[normal_score, scratch_score]` |
| 判断逻辑 | `scratch > normal` → NG，否则 OK |
| 目标芯片 | ESP32-P4 |

**注意**：训练、量化、ESP32 端推理三方的预处理必须完全一致。

---

## 三、数据准备

文件夹结构：

```text
dataset/
  train/
    normal/         ← 正常样品图，建议 ≥100 张
    scratch/        ← 划痕样品图，建议 ≥100 张
  val/
    normal/         ← 验证用正常图
    scratch/        ← 验证用划痕图
```

图片要求：
- 格式：`.jpg` 或 `.png`
- 内容：样品在画面中位置、大小尽量和实际产线一致
- 多样性：不同角度、不同光照、不同划痕形态都要覆盖
- 标签：按文件夹分类，不需要标注框

如果只有 `train` 没有 `val`，脚本会自动从 `train` 划分 20% 作为验证集。

---

## 四、快速开始（Colab）

### 4.1 准备数据

1. 把 `dataset/` 文件夹打包成 `dataset.zip`
2. 上传到 Google Colab 的 `/content/`

### 4.2 上传并运行脚本

把项目里的 `tools/visual_classification_colab.py` 上传到 Colab，然后运行：

```python
!pip install esp-ppq onnx pillow scikit-learn -q
!python visual_classification_colab.py \
    --zip_path /content/dataset.zip \
    --img_size 320 \
    --epochs 100 \
    --calib_samples 200
```

### 4.3 下载模型

```python
from google.colab import files
files.download('/content/best_model.espdl')
```

---

## 五、训练脚本核心逻辑

模型定义（MobileNetV3-Small）：

```python
import torchvision.models as models
import torch.nn as nn

def create_model(num_classes=2):
    model = models.mobilenet_v3_small(weights=None)
    in_features = model.classifier[0].in_features
    model.classifier = nn.Sequential(
        nn.Linear(in_features, 128),
        nn.Hardswish(inplace=True),
        nn.Dropout(p=0.3, inplace=True),
        nn.Linear(128, num_classes)
    )
    return model
```

预处理（与 ESP32 固件对齐）：

```python
def letterbox_rgb(img, size=320):
    w, h = img.size
    scale = min(size / w, size / h)
    new_w = int(round(w * scale))
    new_h = int(round(h * scale))
    resized = img.resize((new_w, new_h), Image.BILINEAR)
    canvas = Image.new("RGB", (size, size), (114, 114, 114))
    left = int(round((size - new_w) / 2 - 0.1))
    top = int(round((size - new_h) / 2 - 0.1))
    canvas.paste(resized, (left, top))
    return canvas
```

---

## 六、量化脚本核心逻辑

```python
from esp_ppq import QuantizationSettingFactory
from esp_ppq.api import espdl_quantize_onnx

quant_setting = QuantizationSettingFactory.espdl_setting()
quant_setting.quantize_activation_setting.calib_algorithm = "percentile"

graph = espdl_quantize_onnx(
    onnx_import_file="/content/best_model.onnx",
    espdl_export_file="/content/best_model.espdl",
    calib_dataloader=calib_loader,
    calib_steps=128,
    input_shape=[1, 3, 320, 320],
    target="esp32p4",
    num_of_bits=8,
    device="cpu",
    setting=quant_setting,
)
```

---

## 七、ESP32 端部署

### 7.1 替换模型

把下载的 `best_model.espdl` 复制到项目：

```text
F:\AI-detection\model\best_model.espdl
```

然后执行：

```powershell
idf.py fullclean
idf.py build flash monitor
```

### 7.2 固件解析逻辑

新的分类模型输出是 `[1, 2]`，固件需要修改 `ai_vision.cpp`：

1. 检测到输出 shape 为 `[1, 2]` 时，按分类模型解析
2. 读取两个分数：`normal` 和 `scratch`
3. 比较大小，输出 OK/NG
4. 不画框

示例代码：

```cpp
// 假设 score[0] = normal, score[1] = scratch
float normal_score = get_output(0);
float scratch_score = get_output(1);
int class_id = (scratch_score > normal_score) ? 1 : 0;
float score = (class_id == 1) ? scratch_score : normal_score;
```

---

## 八、验收标准

量化完成后，请确认：

1. `.info` 文件中输入 shape 为 `[1, 3, 320, 320]`
2. `.info` 文件中输出 shape 为 `[1, 2]`
3. `export_test_values` 中，划痕图输出 `scratch > normal`
4. 正常图输出 `normal > scratch`

---

## 九、参考资源

1. **ESP-DL 官方 ImageNet 分类示例**
   - https://github.com/espressif/esp-dl/tree/master/models/imagenet_cls
   - 官方 MobileNetV2 分类模型和 ESP32-P4 部署示例

2. **ESP-DL 官方量化教程**
   - https://docs.espressif.com/projects/esp-dl/en/latest/getting_started/readme.html
   - 包含 `How to quantize MobileNetV2`

3. **PyTorch → ESP32-P4 完整教程**
   - https://www.haraldkreuzer.net/en/news/esp32-p4-deep-learning-tutorial-mnist-pytorch-and-esp-dl
   - 从训练到量化到 C++ 部署全流程

4. **ESP-VISION 图像分类 API 文档**
   - https://docs.espressif.com/projects/esp-vision/zh_CN/latest/esp32p4/api-reference/espdl.html
   - `ImageNetCls` 使用说明

---

## 十、交付物

请交付：

1. `best_model.espdl`
2. ONNX 文件 `best_model.onnx`
3. 量化日志或 `.info` 文件
4. 验证集准确率
