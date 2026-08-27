#!/usr/bin/env python3
"""风机仿真模型自检: 在训练集/实时画面跑检测"""
import sys
import cv2
import numpy as np
import torch
import glob
sys.path.insert(0, '/home/developer/yolov5')
_old = torch.load
torch.load = lambda *a, **kw: _old(*a, **{'weights_only': False, **kw})
from models.common import DetectMultiBackend
from utils.general import non_max_suppression

MODEL_PATH = '/home/developer/ros2_ws/output/turbine_sim.pt'
model = DetectMultiBackend(MODEL_PATH, device=torch.device('cuda'), fp16=False)


def detect(img, conf=0.25):
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
    det = non_max_suppression(model(im), conf, 0.45)[0]
    if det is None or len(det) == 0:
        return []
    gain_w = new_w / w0
    gain_h = new_h / h0
    results = []
    for d in det:
        x1, y1, x2, y2, c, cls = d.cpu().numpy()
        results.append({
            'cls': int(cls),
            'conf': float(c),
            'bbox': [int((x1 - left) / gain_w), int((y1 - top) / gain_h),
                     int((x2 - left) / gain_w), int((y2 - top) / gain_h)]
        })
    return results


if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'train'
    if mode == 'train':
        files = sorted(glob.glob('/home/developer/yolov5/datasets/turbine/images/train/*.jpg'))
        det_count = 0
        for f in files[:20]:
            img = cv2.imread(f)
            results = detect(img)
            if results:
                det_count += 1
            print(f'{f.split("/")[-1]}: {len(results)} 检测 ' +
                  ' '.join(f'类{r["cls"]}={r["conf"]:.2f}' for r in results))
        print(f'\n训练集自检: {det_count}/20 张有检测')
    elif mode == 'live':
        img_path = sys.argv[2] if len(sys.argv) > 2 else '/tmp/live_frame.jpg'
        img = cv2.imread(img_path)
        if img is None:
            print(f'图片不存在: {img_path}')
            sys.exit(1)
        results = detect(img)
        print(f'检测数: {len(results)}')
        for r in results:
            print(f'  类{r["cls"]} conf={r["conf"]:.2f} bbox={r["bbox"]}')
