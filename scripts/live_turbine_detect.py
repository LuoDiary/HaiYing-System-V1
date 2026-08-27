#!/usr/bin/env python3
"""
实时Gazebo风机缺陷检测 - 相机多视角扫描, 检测到缺陷立即保存截图
"""
import os
import sys
import time
import math
import random
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')
sys.path.insert(0, '/home/developer/yolov5')

import numpy as np
import cv2
import torch

_old = torch.load
torch.load = lambda *a, **kw: _old(*a, **{'weights_only': False, **kw})
from models.common import DetectMultiBackend
from utils.general import non_max_suppression

sys.path.insert(0, '/tmp')
from collect_turbine_data import set_camera_pose, capture_frame

MODEL_PATH = '/home/developer/ros2_ws/output/turbine_sim.pt'
OUTPUT = '/home/developer/ros2_ws/output/detection_result.jpg'
CLASS_NAMES = ['craze', 'corrosion', 'surface_injure', 'thunderstrike', 'crack', 'hide_craze']
COLORS = [(0,255,0),(255,0,0),(0,0,255),(255,255,0),(255,0,255),(0,255,255)]

# 缺陷世界坐标 (与采集一致), 相机瞄准每个缺陷扫描
from collect_turbine_data import DEFECTS
VIEWS = []
for d in DEFECTS:
    for dist in [6.0, 9.0, 12.0]:
        for az in [0.0, 1.0, -1.0]:
            dx, dy, dz = d[2], d[3], d[4]
            cam_x = dx + dist * math.cos(az)
            cam_y = dy + dist * math.sin(az)
            cam_z = dz
            yaw = math.atan2(dy - cam_y, dx - cam_x)
            VIEWS.append((cam_x, cam_y, cam_z, yaw))

model = DetectMultiBackend(MODEL_PATH, device=torch.device('cuda'), fp16=False)
print('模型加载: turbine_sim.pt (Gazebo仿真风机缺陷检测)')

best_result = None

for vi, (cx, cy, cz, yaw) in enumerate(VIEWS):
    set_camera_pose(cx, cy, cz, yaw)
    time.sleep(1.5)
    img = capture_frame()
    img = capture_frame()  # 第二帧确保新画面
    if img is None:
        continue

    h0, w0 = img.shape[:2]
    r = min(640 / h0, 640 / w0)
    new_h, new_w = int(h0 * r), int(w0 * r)
    dh, dw = 640 - new_h, 640 - new_w
    top, left = dh // 2, dw // 2
    im = cv2.resize(img, (new_w, new_h))
    im = cv2.copyMakeBorder(im, top, dh - top, left, dw - left,
                            cv2.BORDER_CONSTANT, value=(114, 114, 114))
    im = im[:, :, ::-1].transpose(2, 0, 1)
    im = np.ascontiguousarray(im)
    im = torch.from_numpy(im).to(torch.device('cuda')).float() / 255.0
    im = im.unsqueeze(0)
    det = non_max_suppression(model(im), 0.25, 0.45)[0]

    annotated = img.copy()
    n = 0
    if det is not None and len(det):
        gain_w = new_w / w0
        gain_h = new_h / h0
        for d in det:
            x1, y1, x2, y2, conf, cls = d.cpu().numpy()
            x1 = int((x1 - left) / gain_w); y1 = int((y1 - top) / gain_h)
            x2 = int((x2 - left) / gain_w); y2 = int((y2 - top) / gain_h)
            color = COLORS[int(cls) % 6]
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 3)
            label = f'{CLASS_NAMES[int(cls)]} {conf:.2f}'
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
            cv2.rectangle(annotated, (x1, y1 - th - 6), (x1 + tw, y1), color, -1)
            cv2.putText(annotated, label, (x1, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            n += 1
            print(f'视角{vi+1}: 检测到 {CLASS_NAMES[int(cls)]} conf={conf:.2f}')

    if n > 0 and (best_result is None or n > best_result[0]):
        best_result = (n, annotated, vi)
        cv2.imwrite(OUTPUT, annotated)
        print(f'  → 截图已保存 ({n}个缺陷)')

if best_result:
    print(f'\n✅ 检测成功: 视角{best_result[2]+1} 检测到 {best_result[0]} 个风机缺陷')
    print(f'截图: {OUTPUT}')
else:
    print('\n❌ 6个视角均未检测到缺陷')
    sys.exit(1)
