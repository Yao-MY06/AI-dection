#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32-P4 视觉二分类模型训练 + 量化脚本
目标：判断样品是好(normal)还是有划痕(scratch)

用法（Google Colab）：
    1. 上传数据集 ZIP（文件夹结构见下方）
    2. 上传本文件到 Colab
    3. 运行: !python visual_classification_colab.py --data_dir /content/dataset --epochs 100
    4. 下载生成的 best_model.espdl

数据集结构：
    dataset/
      train/
        normal/     ← 正常样品图
        scratch/    ← 划痕样品图
      val/
        normal/
        scratch/
"""

import os
import sys
import argparse
import math
import shutil
import zipfile
from pathlib import Path
from collections import Counter

import numpy as np
from PIL import Image
from sklearn.model_selection import train_test_split

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset
import torchvision.transforms as transforms
import torchvision.models as models

import onnx

# ==================== 命令行参数 ====================
parser = argparse.ArgumentParser(description="Train visual classifier for ESP32-P4")
parser.add_argument("--data_dir", type=str, default="/content/dataset",
                    help="数据集根目录，包含 train/val 子文件夹")
parser.add_argument("--zip_path", type=str, default=None,
                    help="如果提供，会从这个 zip 自动解压到 data_dir")
parser.add_argument("--img_size", type=int, default=320,
                    help="模型输入尺寸")
parser.add_argument("--batch_size", type=int, default=32)
parser.add_argument("--epochs", type=int, default=100)
parser.add_argument("--lr", type=float, default=1e-3)
parser.add_argument("--device", type=str, default="cuda",
                    help="训练设备 cuda 或 cpu")
parser.add_argument("--calib_samples", type=int, default=200,
                    help="INT8 量化校准样本数")
parser.add_argument("--calib_steps", type=int, default=128)
parser.add_argument("--output_name", type=str, default="best_model",
                    help="输出模型文件名前缀")
args = parser.parse_args()

CLASS_NAMES = ["normal", "scratch"]
NUM_CLASSES = len(CLASS_NAMES)

DEVICE = torch.device(args.device if torch.cuda.is_available() else "cpu")
IMG_SIZE = args.img_size
BATCH_SIZE = args.batch_size
EPOCHS = args.epochs
LR = args.lr
DATASET_DIR = args.data_dir

print("=" * 60)
print("ESP32-P4 视觉二分类训练 + 量化")
print("=" * 60)
print(f"Device        : {DEVICE}")
print(f"Image size    : {IMG_SIZE}x{IMG_SIZE}")
print(f"Classes       : {CLASS_NAMES}")
print(f"Epochs        : {EPOCHS}")
print(f"Dataset dir   : {DATASET_DIR}")
print("=" * 60)

# ==================== 自动解压 ZIP ====================
if args.zip_path and Path(args.zip_path).exists():
    extract_dir = Path(DATASET_DIR)
    if extract_dir.exists():
        shutil.rmtree(extract_dir)
    extract_dir.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(args.zip_path, "r") as zf:
        zf.extractall(extract_dir)
    print(f"Extracted {args.zip_path} to {extract_dir}")

# ==================== Dataset ====================
class DefectDataset(Dataset):
    def __init__(self, file_paths, labels, transform=None):
        self.file_paths = file_paths
        self.labels = labels
        self.transform = transform

    def __len__(self):
        return len(self.file_paths)

    def __getitem__(self, idx):
        img = Image.open(self.file_paths[idx]).convert("RGB")
        label = self.labels[idx]
        if self.transform:
            img = self.transform(img)
        return img, label


def letterbox_rgb(img, size=IMG_SIZE):
    """与 ESP32 固件完全一致的 letterbox 预处理。"""
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


def build_transforms(is_train=True):
    """构建训练/验证预处理。"""
    base = [transforms.Lambda(letterbox_rgb)]
    if is_train:
        base.extend([
            transforms.RandomHorizontalFlip(p=0.5),
            transforms.RandomRotation(degrees=10),
            transforms.ColorJitter(brightness=0.15, contrast=0.15),
        ])
    base.extend([
        transforms.ToTensor(),  # -> [0, 1]
    ])
    return transforms.Compose(base)


# 扫描数据集
all_files = []
all_labels = []
for cls_idx, cls_name in enumerate(CLASS_NAMES):
    for split in ["train", "val"]:
        cls_dir = Path(DATASET_DIR) / split / cls_name
        if not cls_dir.exists():
            continue
        for fname in sorted(cls_dir.glob("*.png")) + sorted(cls_dir.glob("*.jpg")) + sorted(cls_dir.glob("*.jpeg")):
            all_files.append(str(fname))
            all_labels.append(cls_idx)

if len(all_files) == 0:
    print("[ERROR] 没有找到任何图片！请检查 --data_dir 和数据集结构")
    sys.exit(1)

print(f"\nTotal images : {len(all_files)}")
print(f"Class distribution: {dict((CLASS_NAMES[i], v) for i, v in Counter(all_labels).items())}")

# 划分训练/验证（如果只有 train，按 8:2 自动划分）
train_files, val_files, train_labels, val_labels = [], [], [], []
for split in ["train", "val"]:
    split_files = []
    split_labels = []
    for cls_idx, cls_name in enumerate(CLASS_NAMES):
        cls_dir = Path(DATASET_DIR) / split / cls_name
        if not cls_dir.exists():
            continue
        for fname in sorted(cls_dir.glob("*.png")) + sorted(cls_dir.glob("*.jpg")) + sorted(cls_dir.glob("*.jpeg")):
            split_files.append(str(fname))
            split_labels.append(cls_idx)
    if split == "train":
        train_files, train_labels = split_files, split_labels
    else:
        val_files, val_labels = split_files, split_labels

if len(val_files) == 0 and len(train_files) > 0:
    print("[INFO] 未找到 val 目录，自动从 train 划分 8:2 作为验证集")
    train_files, val_files, train_labels, val_labels = train_test_split(
        train_files, train_labels, test_size=0.2, stratify=train_labels, random_state=42
    )

train_dataset = DefectDataset(train_files, train_labels, transform=build_transforms(is_train=True))
val_dataset = DefectDataset(val_files, val_labels, transform=build_transforms(is_train=False))

train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True, num_workers=2, pin_memory=True)
val_loader = DataLoader(val_dataset, batch_size=BATCH_SIZE, shuffle=False, num_workers=2, pin_memory=True)

# ==================== Model ====================
def create_model(num_classes=2):
    """MobileNetV3-Small RGB 分类模型。"""
    model = models.mobilenet_v3_small(weights=None)
    # 修改分类头
    in_features = model.classifier[0].in_features
    model.classifier = nn.Sequential(
        nn.Linear(in_features, 128),
        nn.Hardswish(inplace=True),
        nn.Dropout(p=0.3, inplace=True),
        nn.Linear(128, num_classes)  # 输出 logits，不加 Softmax
    )
    return model


model = create_model(NUM_CLASSES).to(DEVICE)
total_params = sum(p.numel() for p in model.parameters())
print(f"\nModel parameters: {total_params / 1e6:.2f} M")

# ==================== 类别不平衡处理 ====================
from sklearn.utils.class_weight import compute_class_weight
class_weights = compute_class_weight("balanced", classes=np.unique(train_labels), y=train_labels)
class_weights_tensor = torch.tensor(class_weights, dtype=torch.float32).to(DEVICE)
print(f"Class weights: {class_weights}")

# ==================== Training ====================
criterion = nn.CrossEntropyLoss(weight=class_weights_tensor)
optimizer = optim.AdamW(model.parameters(), lr=LR, weight_decay=1e-4)
scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode="max", factor=0.5, patience=10, verbose=True)

best_acc = 0.0
best_pth = f"/content/{args.output_name}.pth"

print("\n[Training started...]")
for epoch in range(EPOCHS):
    # Training
    model.train()
    train_loss = 0.0
    train_correct = 0
    train_total = 0
    for imgs, labels in train_loader:
        imgs, labels = imgs.to(DEVICE), labels.to(DEVICE)
        optimizer.zero_grad()
        outputs = model(imgs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()
        train_loss += loss.item()
        _, predicted = torch.max(outputs, 1)
        train_correct += (predicted == labels).sum().item()
        train_total += labels.size(0)
    train_acc = train_correct / train_total

    # Validation
    model.eval()
    val_loss = 0.0
    val_correct = 0
    val_total = 0
    with torch.no_grad():
        for imgs, labels in val_loader:
            imgs, labels = imgs.to(DEVICE), labels.to(DEVICE)
            outputs = model(imgs)
            loss = criterion(outputs, labels)
            val_loss += loss.item()
            _, predicted = torch.max(outputs, 1)
            val_correct += (predicted == labels).sum().item()
            val_total += labels.size(0)
    val_acc = val_correct / val_total
    scheduler.step(val_acc)

    if val_acc > best_acc:
        best_acc = val_acc
        torch.save(model.state_dict(), best_pth)
        print(f"Epoch [{epoch+1}/{EPOCHS}] - New best model saved! Val Acc: {val_acc:.4f}")
    else:
        print(f"Epoch [{epoch+1}/{EPOCHS}] - Train Acc: {train_acc:.4f}, Val Acc: {val_acc:.4f}")

print(f"\nTraining complete. Best validation accuracy: {best_acc:.4f}")

# ==================== Export ONNX ====================
model.load_state_dict(torch.load(best_pth))
model.eval()
model.to("cpu")

dummy_input = torch.randn(1, 3, IMG_SIZE, IMG_SIZE)
onnx_path = f"/content/{args.output_name}.onnx"
torch.onnx.export(
    model,
    dummy_input,
    onnx_path,
    input_names=["input"],
    output_names=["output"],
    dynamic_axes=None,
    opset_version=11,
    do_constant_folding=True,
)
print(f"ONNX exported to: {onnx_path}")

onnx_model = onnx.load(onnx_path)
onnx.checker.check_model(onnx_model)
print("ONNX validation passed!")

# ==================== Quantization ====================
print("\n[Quantization started...]")
try:
    from esp_ppq import QuantizationSettingFactory
    from esp_ppq.api import espdl_quantize_onnx
except ImportError:
    print("Installing esp-ppq...")
    os.system("pip install esp-ppq -q")
    from esp_ppq import QuantizationSettingFactory
    from esp_ppq.api import espdl_quantize_onnx


class CalibDataset(Dataset):
    def __init__(self, file_paths, max_images=args.calib_samples):
        self.files = file_paths[:max_images]

    def __len__(self):
        return len(self.files)

    def __getitem__(self, idx):
        img = Image.open(self.files[idx]).convert("RGB")
        img = letterbox_rgb(img)
        arr = np.asarray(img, dtype=np.float32) / 255.0
        arr = np.transpose(arr, (2, 0, 1))  # CHW
        return torch.from_numpy(arr)


calib_dataset = CalibDataset(val_files, max_images=args.calib_samples)
calib_loader = DataLoader(calib_dataset, batch_size=1, shuffle=False)
print(f"Calibration samples: {len(calib_dataset)}")


def collate_fn(batch):
    if isinstance(batch, list):
        return torch.stack(batch).to("cpu")
    return batch.to("cpu")


quant_setting = QuantizationSettingFactory.espdl_setting()
quant_setting.quantize_activation_setting.calib_algorithm = "percentile"

espdl_path = f"/content/{args.output_name}.espdl"
graph = espdl_quantize_onnx(
    onnx_import_file=onnx_path,
    espdl_export_file=espdl_path,
    calib_dataloader=calib_loader,
    calib_steps=min(args.calib_steps, len(calib_dataset)),
    input_shape=[1, 3, IMG_SIZE, IMG_SIZE],
    inputs=None,
    target="esp32p4",
    num_of_bits=8,
    collate_fn=collate_fn,
    device="cpu",
    setting=quant_setting,
    error_report=True,
    skip_export=False,
    export_config=True,
    export_test_values=True,
    test_output_names=["output"],
    verbose=1,
)

print(f"\nESPDL model saved to: {espdl_path}")
print(f"File size: {os.path.getsize(espdl_path) / 1024:.1f} KB")

# 下载提示
print("\n请运行以下代码下载模型到本地：")
print(f"from google.colab import files; files.download('{espdl_path}')")
