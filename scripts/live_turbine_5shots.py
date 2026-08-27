#!/usr/bin/env python3
"""
Gazebo风机缺陷检测 - 保存5张检测结果截图 (不同缺陷类型/视角)
"""
import os
import sys
import time
import math
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
from collect_turbine_data import set_camera_pose, capture_frame, DEFECTS

MODEL_PATH = '/home/developer/ros2_ws/output/turbine_sim.pt'
OUT_DIR = '/home/developer/ros2_ws/output'
CLASS_NAMES = ['craze', 'corrosion', 'surface_injure', 'thunderstrike', 'crack', 'hide_craze']
COLORS = [(0,255,0),(255,0,0),(0,0,255),(255,255,0),(255,0,255),(0,255,255)]

model = DetectMultiBackend(MODEL_PATH, device=torch.device('cuda'), fp16=False)
print('模型: turbine_sim.pt | 目标: 保存5张不同缺陷检测截图')

# 每种缺陷类型保存最多3张不同视角
shots = {}  # (class_name, dist, az) -> (conf, annotated_img)
MAX_PER_CLASS = 3

def detect_and_annotate(img):
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
    results = []
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
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
            cv2.rectangle(annotated, (x1, y1 - th - 6), (x1 + tw, y1), color, -1)
            cv2.putText(annotated, label, (x1, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            results.append((int(cls), float(conf)))
    return results, annotated


# 扫描每个缺陷的多个视角
for d in DEFECTS:
    for dist in [5.0, 7.0, 10.0]:
        for az in [0.0, 0.8, -0.8]:
            dx, dy, dz = d[2], d[3], d[4]
            cam_x = dx + dist * math.cos(az)
            cam_y = dy + dist * math.sin(az)
            cam_z = dz
            yaw = math.atan2(dy - cam_y, dx - cam_x)

            set_camera_pose(cam_x, cam_y, cam_z, yaw)
            time.sleep(1.3)
            img = capture_frame()
            img = capture_frame()
            if img is None:
                continue

            results, annotated = detect_and_annotate(img)
            for cls, conf in results:
                name = CLASS_NAMES[cls]
                key = (name, dist, az)
                class_count = sum(1 for k in shots if k[0] == name)
                # 每种缺陷最多MAX_PER_CLASS张, 同一视角取最高置信度
                if class_count < MAX_PER_CLASS or key in shots:
                    if key not in shots or conf > shots[key][0]:
                        shots[key] = (conf, annotated)
                        print(f'{name}: conf={conf:.2f} (距离{dist}m, 方位{az:.1f}rad)')

# 按置信度排序保存5张
sorted_shots = sorted(shots.items(), key=lambda kv: kv[1][0], reverse=True)
print(f'\n共 {len(sorted_shots)} 张候选截图')

for i, (key, (conf, img)) in enumerate(sorted_shots[:5]):
    name, dist, az = key
    out = os.path.join(OUT_DIR, f'detection_result_{i+1}.jpg')
    cv2.imwrite(out, img)
    print(f'  [{i+1}] {out} <- {name} conf={conf:.2f} (距离{dist}m)')

print('\n完成!')
