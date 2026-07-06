# ESP32-P4 Visual Classification Delivery

This package switches the vision model from YOLO detection to whole-image binary classification.

## Files

Primary output directory:

```text
C:\Users\Joe\Documents\Codex\classification_esp32p4
```

Main files:

```text
best_classification_esp32p4.espdl   # deploy this to firmware model/best_model.espdl
best_classification.onnx            # PC/reference model
best_classification_esp32p4.info    # ESPDL structure and exponent report
classification_report.json          # training, test, quantization summary
training_history.csv                # epoch history
classification_manifest.csv         # YOLO labels converted to image labels
train_visual_classification_esp32p4_local.py
```

## Model Spec

Task:

```text
normal / scratch whole-image classification
```

Classes:

```text
0 = normal
1 = scratch
```

Preprocess:

```text
RGB
letterbox to 320x320
fill color = 114,114,114
divide by 255
no ImageNet normalization
```

ESPDL checked output:

```text
input  : INT8 [1,320,320,3], exponent=-7
output : INT8 [1,2], exponent=-6
return : logits
```

The two output values are logits, not YOLO scores. Compare class 1 against class 0:

```cpp
normal_logit  = output[0]
scratch_logit = output[1]
is_scratch = scratch_logit > normal_logit
```

Softmax is optional for display only. For OK/NG, comparing logits is enough.

## Training Result

Dataset was derived from the existing YOLO dataset:

```text
If an image label file contains class 1, the whole image is scratch.
Otherwise, the whole image is normal.
```

Counts:

```text
train: total=262 normal=143 scratch=119
val  : total=31  normal=17  scratch=14
test : total=35  normal=19  scratch=16
```

Validation:

```text
accuracy    = 0.9677
normal_acc  = 1.0000
scratch_acc = 0.9286
confusion   = [[17,0],[1,13]]
```

Board screenshot PC-side predictions:

```text
PIC0335.png -> scratch 0.9809
PIC0336.png -> scratch 0.8268
PIC0337.png -> scratch 0.8248
```

## ESP32-P4 Pitfall Checks

The local script includes the previous fixes:

```text
target=esp32p4
P4 per-channel override disabled before export
PER_CHANNEL config count checked before export
.info multi-exponent tensors checked after export
output shape checked through .info
legacy torch ONNX exporter used, no onnxscript dependency needed
```

Quantization report:

```text
per_channel_count = 0
multi_exponent_count = 0
ESPDL SHA256 = efba64b1b6261b23e47044bd8adade30997f60a18428911d7792d357162463a6
```

## Firmware Requirement

The firmware parser must support classification output `[1,2]`.

Expected logic:

```cpp
// For model output [1,2]:
float normal = read_output_value(output_tensor, 0);
float scratch = read_output_value(output_tensor, 1);

bool detected = scratch > normal;
class_name = detected ? "scratch" : "normal";
score = detected ? scratch : normal;
```

Do not run YOLO box parsing for this model. It has no boxes.

## Deployment

Copy:

```text
best_classification_esp32p4.espdl
```

to the firmware model path and rename it if the project expects:

```text
model/best_model.espdl
```

Then rebuild from a clean state:

```powershell
idf.py fullclean
idf.py build flash monitor
```

Expected serial log should show one model output with shape `[1,2]` or equivalent `1x2`, not YOLO `[1,6,2100]` or two YOLO tensors.

## Re-run Command

```powershell
& "D:\anaconda3\Scripts\conda.exe" --no-plugins run -n espdl python train_visual_classification_esp32p4_local.py --epochs 45 --batch_size 24 --patience 10 --model tiny_relu --output_dir "C:\Users\Joe\Documents\Codex\classification_esp32p4"
```
