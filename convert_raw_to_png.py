import numpy as np
import cv2
import os

WIDTH = 844
HEIGHT = 440

INPUT_DIR = r'D:\pictures\original'
OUTPUT_DIR = r'D:\pictures\process'
os.makedirs(OUTPUT_DIR, exist_ok=True)

print(f"扫描文件夹: {INPUT_DIR}")
all_files = os.listdir(INPUT_DIR)
print(f"共发现 {len(all_files)} 个条目")

# ===== 字节序设置 =====
# 如果转换出的颜色不正常，请将 'little' 改为 'big'
ENDIAN = 'little'   # 'little' 或 'big'

for filename in all_files:
    if not filename.lower().endswith('.raw'):
        continue

    filepath = os.path.join(INPUT_DIR, filename)
    print(f"\n检查: {filename}")

    with open(filepath, 'rb') as f:
        file_bytes = f.read()

    # 按照指定的字节序读取
    dtype = '<u2' if ENDIAN == 'little' else '>u2'
    raw_data = np.frombuffer(file_bytes, dtype=dtype)
    
    print(f"  数据点数: {raw_data.size}, 期望: {WIDTH*HEIGHT}")
    print(f"  原始值范围: min={raw_data.min()}, max={raw_data.max()}")

    if raw_data.size != WIDTH * HEIGHT:
        print(f"  [跳过] 大小不匹配")
        continue

    raw_data = raw_data.reshape((HEIGHT, WIDTH))

    # 确保为小端序用于 OpenCV (cv2.COLOR_BGR5652BGR 需要小端输入)
    if ENDIAN == 'big':
        raw_data = raw_data.byteswap()  # 转成小端序
        print("  已从大端序转换为小端序")
    
    # 转换：将 uint16 数组 view 成 (H, W, 2) 的 uint8 图像
    img_bgr = cv2.cvtColor(raw_data.view(np.uint8).reshape(HEIGHT, WIDTH, 2),
                           cv2.COLOR_BGR5652BGR)

    # 保存
    base_name = os.path.splitext(filename)[0]
    out_path = os.path.join(OUTPUT_DIR, base_name + '.png')
    cv2.imwrite(out_path, img_bgr)
    print(f"  已保存: {out_path}")

print("\n所有图像处理完毕！")