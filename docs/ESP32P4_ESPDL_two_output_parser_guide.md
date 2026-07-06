# ESP32-P4 ESPDL 两输出模型固件解析器修改教程

## 1. 当前问题结论

现在的问题已经不是摄像头、烧录、LCD 蓝屏、PER_CHANNEL 或 Split 节点崩溃。

前面几个问题的状态：

- ESP32-P4 板子可以启动。
- 模型可以加载。
- 摄像头有画面，不是黑图。
- 新导出的 ESPDL 已经不再报 PER_CHANNEL。
- no-Split 模型可以避开 Split 加载崩溃。

现在的核心问题是：

**YOLO 最后一层把 box 坐标和类别分数合并成同一个输出 tensor，导致 ESPDL 量化时用同一个 exponent 处理两种范围完全不同的数据。**

旧输出结构：

```text
output0: [1, 6, 2100]

channel 0-3: box 坐标，范围大约 0..320
channel 4-5: normal / scratch 概率，范围大约 0..1
```

板子日志里曾经看到：

```text
shape=1x6x2100 dtype=int8 exp=1 out_max=254
cand=0 valid=0 best_n=0.000 best_s=0.000
```

`exp=1` 对 box 坐标还算合理，但对 `0..1` 的类别概率非常不合理，会把类别分数压成 0。

所以这次的主线方案是：

**模型导出成两个输出：boxes 和 scores；固件 parser 分别读取两个输出。**

新目标输出结构：

```text
boxes:  [1, 4, 2100]
scores: [1, 2, 2100]
```

这样 ESPDL 可以分别给 boxes 和 scores 设置量化尺度，类别分数不会再被 box 坐标的尺度污染。

---

## 2. 需要替换的模型文件

新 ESPDL 模型位置：

```text
D:\CodexOutputs\esp32p4_espdl\washer_yolov8n_320_no_split_two_outputs_esp32p4_per_tensor.espdl
```

复制到固件项目：

```powershell
Copy-Item `
  "D:\CodexOutputs\esp32p4_espdl\washer_yolov8n_320_no_split_two_outputs_esp32p4_per_tensor.espdl" `
  "D:\ESP-IDF\ESP_AI_LIBS\AI-detection\model\best_model.espdl" `
  -Force
```

注意：板子是 **ESP32-P4**，所以后续构建目标必须是：

```powershell
idf.py set-target esp32p4
```

---

## 3. 需要修改的固件文件

目标文件：

```text
D:\ESP-IDF\ESP_AI_LIBS\AI-detection\main\ai_vision.cpp
```

旧代码通常只有一个解析函数：

```cpp
static bool parse_yolo_output(dl::TensorBase *out_tensor, ai_detection_t *det)
```

它假设输出是单 tensor：

```text
[1, 6, 2100]
```

现在要保留这个旧函数作为回退，同时新增两输出解析逻辑。

---

## 4. 新增 tensor 形状识别函数

把下面函数放在 `parse_yolo_output(...)` 前面或后面都可以，但必须在调用它之前已经声明。

```cpp
static bool detect_yolo_tensor_layout(dl::TensorBase *tensor,
                                      int expected_channels,
                                      int *anchors,
                                      bool *channels_first)
{
    if (!tensor || !anchors || !channels_first) return false;

    std::vector<int> shape = tensor->get_shape();

    if (shape.size() == 3) {
        if (shape[1] == expected_channels) {
            *anchors = shape[2];
            *channels_first = true;
            return *anchors > 0;
        }
        if (shape[2] == expected_channels) {
            *anchors = shape[1];
            *channels_first = false;
            return *anchors > 0;
        }
    }

    if (shape.size() == 2) {
        if (shape[0] == expected_channels) {
            *anchors = shape[1];
            *channels_first = true;
            return *anchors > 0;
        }
        if (shape[1] == expected_channels) {
            *anchors = shape[0];
            *channels_first = false;
            return *anchors > 0;
        }
    }

    if (tensor->get_size() % expected_channels == 0) {
        *anchors = tensor->get_size() / expected_channels;
        *channels_first = true;
        return *anchors > 0;
    }

    return false;
}
```

这个函数的作用：

- 遇到 `[1,4,2100]`，识别成 boxes。
- 遇到 `[1,2,2100]`，识别成 scores。
- 同时兼容 `[1,2100,4]` 和 `[1,2100,2]`。

---

## 5. 新增两输出 YOLO 解析函数

继续在 `ai_vision.cpp` 里新增：

```cpp
static bool parse_yolo_outputs(dl::TensorBase *box_tensor,
                               dl::TensorBase *score_tensor,
                               ai_detection_t *det)
{
    if (!box_tensor || !score_tensor || !det) return false;
    *det = {};

    int box_anchors = 0;
    int score_anchors = 0;
    bool box_cf = true;
    bool score_cf = true;

    if (!detect_yolo_tensor_layout(box_tensor, 4, &box_anchors, &box_cf)) return false;
    if (!detect_yolo_tensor_layout(score_tensor, 2, &score_anchors, &score_cf)) return false;
    if (box_anchors != score_anchors) return false;

    auto get_box = [&](int anchor, int channel) -> float {
        int index = box_cf ? (channel * box_anchors + anchor) : (anchor * 4 + channel);
        return ai_tensor_value_as_float(box_tensor, index);
    };

    auto get_score = [&](int anchor, int channel) -> float {
        int index = score_cf ? (channel * score_anchors + anchor) : (anchor * 2 + channel);
        return ai_tensor_value_as_float(score_tensor, index);
    };

    float best_score = 0.0f;
    int best_anchor = -1;
    int best_class = 0;
    float best_normal = 0.0f;
    float best_scratch = 0.0f;

    for (int i = 0; i < score_anchors; ++i) {
        float normal = maybe_sigmoid_score(get_score(i, 0));
        float scratch = maybe_sigmoid_score(get_score(i, 1));
        if (!isfinite(normal) || !isfinite(scratch)) continue;

        int class_id = (scratch > normal) ? 1 : 0;
        float score = (class_id == 1) ? scratch : normal;

        if (score > best_score) {
            best_score = score;
            best_anchor = i;
            best_class = class_id;
            best_normal = normal;
            best_scratch = scratch;
        }
    }

    if (best_anchor < 0 || best_score < AI_DET_CONF_THRESHOLD) {
        return false;
    }

    float cx = get_box(best_anchor, 0);
    float cy = get_box(best_anchor, 1);
    float bw = get_box(best_anchor, 2);
    float bh = get_box(best_anchor, 3);

    if (!isfinite(cx) || !isfinite(cy) || !isfinite(bw) || !isfinite(bh) || bw <= 0.0f || bh <= 0.0f) {
        return false;
    }

    if (cx <= 2.0f && cy <= 2.0f && bw <= 2.0f && bh <= 2.0f) {
        cx *= AI_INPUT_W;
        bw *= AI_INPUT_W;
        cy *= AI_INPUT_H;
        bh *= AI_INPUT_H;
    }

    float x1 = (cx - bw * 0.5f - s_letterbox_pad_x) / s_letterbox_scale;
    float y1 = (cy - bh * 0.5f - s_letterbox_pad_y) / s_letterbox_scale;
    float x2 = (cx + bw * 0.5f - s_letterbox_pad_x) / s_letterbox_scale;
    float y2 = (cy + bh * 0.5f - s_letterbox_pad_y) / s_letterbox_scale;

    x1 = clamp_float(x1, 0.0f, (float)(CAM_W - 1));
    y1 = clamp_float(y1, 0.0f, (float)(CAM_H - 1));
    x2 = clamp_float(x2, 0.0f, (float)(CAM_W - 1));
    y2 = clamp_float(y2, 0.0f, (float)(CAM_H - 1));

    int left = round_to_int(x1);
    int top = round_to_int(y1);
    int right = round_to_int(x2);
    int bottom = round_to_int(y2);

    if (right <= left || bottom <= top) {
        return false;
    }

    det->present = true;
    det->class_id = best_class;
    det->score = best_score;
    det->normal_score = best_normal;
    det->scratch_score = best_scratch;
    det->x = left;
    det->y = top;
    det->w = right - left;
    det->h = bottom - top;
    return true;
}
```

---

## 6. 修改推理输出读取逻辑

在 `ai_vision_task(...)` 里找到类似代码：

```cpp
auto &outputs = yolo_model->get_outputs();
if (!outputs.empty()) {
    dl::TensorBase *out_tensor = outputs.begin()->second;
    if (out_tensor && out_tensor->get_element_ptr()) {
        ai_detection_t det = {};
        system_event_t ev = {};

        if (!parse_yolo_output(out_tensor, &det)) {
            ...
        }
    }
}
```

把读取输出的部分改成：

```cpp
auto &outputs = yolo_model->get_outputs();
if (!outputs.empty()) {
    dl::TensorBase *single_tensor = nullptr;
    dl::TensorBase *box_tensor = nullptr;
    dl::TensorBase *score_tensor = nullptr;

    for (auto &kv : outputs) {
        dl::TensorBase *tensor = kv.second;
        if (!tensor || !tensor->get_element_ptr()) continue;

        int anchors = 0;
        bool channels_first = true;

        if (!box_tensor && detect_yolo_tensor_layout(tensor, 4, &anchors, &channels_first)) {
            box_tensor = tensor;
        } else if (!score_tensor && detect_yolo_tensor_layout(tensor, 2, &anchors, &channels_first)) {
            score_tensor = tensor;
        } else if (!single_tensor) {
            single_tensor = tensor;
        }
    }

    ai_detection_t det = {};
    system_event_t ev = {};

    bool parsed = false;
    if (box_tensor && score_tensor) {
        parsed = parse_yolo_outputs(box_tensor, score_tensor, &det);
    } else if (single_tensor) {
        parsed = parse_yolo_output(single_tensor, &det);
    }

    if (!parsed) {
        g_sample_present = false;
        g_sample_roi = {};
        g_ai_ng_pct = 0.0f;
        g_ai_ok_pct = 0.0f;
        ev.type = EVENT_AI_NO_DETECTION;
        snprintf(ev.log_msg, sizeof(ev.log_msg), "\n[AI] NO SAMPLE");
        if (xQueueSend(ui_event_queue, &ev, pdMS_TO_TICKS(20)) != pdPASS) {
            ESP_LOGW(TAG, "UI event queue full, AI no-detection event dropped!");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
        continue;
    }

    // parsed 成功后，保留原来的 ROI、OK/NG 百分比、事件上报逻辑
}
```

注意：

这里的关键不是 tensor 名字，而是 tensor shape。

不要强依赖输出名一定叫 `boxes` 或 `scores`，因为 ESPDL 导出后名字可能变化。最稳妥的是：

- `4 通道` 当 boxes
- `2 通道` 当 scores
- 找不到两输出时，再回退旧单输出 parser

---

## 7. 建议增加输出日志

为了确认板子到底拿到了几个输出，建议在遍历 outputs 时加日志。

可以放在 `for (auto &kv : outputs)` 里面：

```cpp
auto shape = tensor->get_shape();
if (ai_run_cnt == 1) {
    if (shape.size() == 3) {
        ESP_LOGI(TAG, "[AI] output %s shape=[%d,%d,%d] dtype=%s exp=%d",
                 kv.first.c_str(),
                 shape[0], shape[1], shape[2],
                 tensor->get_dtype_string(),
                 tensor->get_exponent());
    } else {
        ESP_LOGI(TAG, "[AI] output %s dims=%u size=%d dtype=%s exp=%d",
                 kv.first.c_str(),
                 (unsigned)shape.size(),
                 tensor->get_size(),
                 tensor->get_dtype_string(),
                 tensor->get_exponent());
    }
}
```

理想日志应该接近：

```text
[AI] output boxes  shape=[1,4,2100] dtype=int8 exp=...
[AI] output scores shape=[1,2,2100] dtype=int8 exp=...
```

如果仍然只看到：

```text
[AI] output output0 shape=[1,6,2100] dtype=int8 exp=1
```

说明 ESPDL 导出器没有真正保留两输出，这时需要回到转换脚本继续处理 ONNX 导出。

---

## 8. 编译和烧录

进入项目：

```powershell
cd D:\ESP-IDF\ESP_AI_LIBS\AI-detection
```

确认目标芯片：

```powershell
idf.py set-target esp32p4
```

建议完整清理后重新编译：

```powershell
idf.py fullclean
idf.py build flash monitor
```

---

## 9. 上板后重点看什么

重点看这几类日志。

### 9.1 不应该再出现的日志

不应该再出现：

```text
PER_CHANNEL quantization
Split
FbsLoader not aligned
Guru Meditation during dl::Model::load
```

如果这些又出现，说明用错模型文件，或者 CMake 嵌入模型的方式回退了。

### 9.2 应该确认输出数量

最好看到两个输出：

```text
shape=[1,4,2100]
shape=[1,2,2100]
```

其中：

- `[1,4,2100]` 是 boxes。
- `[1,2,2100]` 是 scores。

### 9.3 类别分数是否恢复

之前坏的表现是：

```text
best_n=0.000 best_s=0.000
```

修好后，至少应该能看到 normal 或 scratch 有非零分数。

如果还是全 0，需要继续检查：

- `scores` tensor 的 exponent 是否合理。
- 固件是否真的走了 `parse_yolo_outputs(...)`。
- ESPDL 是否实际只有一个 `output0`。

---

## 10. 结论

这一步不是重新训练模型。

当前优先级是：

1. 使用 no-Split、PER_TENSOR、two-output 的 ESPDL 模型。
2. 固件 parser 支持 `boxes + scores` 两输出。
3. 上板确认 `scores` 分数不再全 0。

只有当两输出 parser 已经确认正常，但识别效果仍然差时，才需要回到训练集、阈值、输入分辨率或模型结构继续优化。

