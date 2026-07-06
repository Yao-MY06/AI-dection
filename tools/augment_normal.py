#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
正常样本严重不足时的救急增强脚本
用法：
    python augment_normal.py \
        --src_dir dataset/normal \
        --out_dir dataset/normal_augmented \
        --num_target 200

把 2 张 normal 通过 heavy augmentation 扩展到 200 张，弥补数据不平衡。
注意：这只是临时方案，有条件请务必补拍真实 normal 样本！
"""

import os
import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageEnhance
import albumentations as A

parser = argparse.ArgumentParser()
parser.add_argument('--src_dir', type=str, required=True, help='原始 normal 图片文件夹（只有几张图）')
parser.add_argument('--out_dir', type=str, required=True, help='增强后输出文件夹')
parser.add_argument('--num_target', type=int, default=200, help='目标生成数量')
args = parser.parse_args()

os.makedirs(args.out_dir, exist_ok=True)

# 读取原始 normal 图
src_files = []
for ext in ('*.jpg', '*.jpeg', '*.png', '*.bmp', '*.webp'):
    src_files.extend(Path(args.src_dir).glob(ext))
    src_files.extend(Path(args.src_dir).glob(ext.upper()))

src_files = sorted(list(set(src_files)))
if len(src_files) == 0:
    raise ValueError(f"在 {args.src_dir} 中没有找到图片！")

print(f"找到 {len(src_files)} 张原始 normal 图，目标生成 {args.num_target} 张...")

# Heavy augmentation pipeline（针对工业质检，模拟各种正常工况）
transform = A.Compose([
    A.HorizontalFlip(p=0.5),
    A.VerticalFlip(p=0.3),
    A.RandomRotate90(p=0.5),
    A.Rotate(limit=30, p=0.8, border_mode=cv2.BORDER_REFLECT_101),
    A.ShiftScaleRotate(shift_limit=0.1, scale_limit=0.15, rotate_limit=15, p=0.8),
    A.RandomResizedCrop(height=128, width=128, scale=(0.7, 1.0), ratio=(0.9, 1.1), p=0.7),
    A.OneOf([
        A.RandomBrightnessContrast(brightness_limit=0.3, contrast_limit=0.3, p=1.0),
        A.CLAHE(clip_limit=4.0, p=1.0),
        A.RandomGamma(gamma_limit=(60, 140), p=1.0),
    ], p=0.9),
    A.OneOf([
        A.GaussNoise(var_limit=(20.0, 100.0), p=1.0),
        A.ISONoise(color_shift=(0.05, 0.15), intensity=(0.1, 0.5), p=1.0),
    ], p=0.7),
    A.OneOf([
        A.MotionBlur(blur_limit=5, p=1.0),
        A.MedianBlur(blur_limit=5, p=1.0),
        A.GaussianBlur(blur_limit=5, p=1.0),
    ], p=0.4),
    A.CoarseDropout(
        num_holes_range=(1, 3),
        hole_height_range=(8, 16),
        hole_width_range=(8, 16),
        fill_value=128,
        p=0.3
    ),
], p=1.0)

generated = 0
img_idx = 0
max_attempts = args.num_target * 10
attempts = 0

while generated < args.num_target and attempts < max_attempts:
    attempts += 1
    src_path = str(src_files[img_idx % len(src_files)])
    img = cv2.imread(src_path)
    if img is None:
        img_idx += 1
        continue
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    try:
        augmented = transform(image=img)['image']
    except Exception as e:
        print(f"Augment failed: {e}")
        img_idx += 1
        continue

    # 保存为灰度（和训练一致）
    aug_gray = cv2.cvtColor(augmented, cv2.COLOR_RGB2GRAY)
    out_path = os.path.join(args.out_dir, f"normal_aug_{generated:04d}.png")
    cv2.imwrite(out_path, aug_gray)
    generated += 1
    img_idx += 1

print(f"✅ 生成完成: {generated} 张图已保存到 {args.out_dir}")
print("⚠️  提醒：这只是临时方案，有条件请务必补拍真实 normal 样本！")
