#!/usr/bin/env python3
"""
风机缺陷数据采集 - Gazebo渲染图 + 缺陷坐标投影自动标注
相机在风机周围随机位姿拍摄, 已知缺陷世界坐标投影到像素生成YOLO标签
"""
import os
import sys
import time
import math
import random
import json
import subprocess
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

import numpy as np
import cv2

# Gazebo相机参数 (turbine_cam: 1920x1080, HFOV 1.204)
IMG_W, IMG_H = 1920, 1080
HFOV = 1.204
FX = (IMG_W / 2) / math.tan(HFOV / 2)
FY = FX
CX, CY = IMG_W / 2, IMG_H / 2

# 缺陷世界坐标 + 类别 + 近似尺寸(米)
# 模型spawn在(30,5,0)
DEFECTS = [
    # (name, class_id, x, y, z, size_x, size_y) size为世界尺寸(米)
    ('corrosion', 1, 32.55, 5.0, 20.0, 4.0, 3.0),     # 塔筒腐蚀
    ('crack1',    4, 37.3, 5.21, 78.0, 3.0, 1.5),     # 叶片1裂纹 (3m大块)
    ('crack2',    4, 36.8, 4.79, 83.0, 2.5, 1.2),     # 叶片1裂纹2
    ('surface',   2, 37.0, 11.37, 47.18, 2.4, 2.4),   # 叶片2表面损伤 (1.2m球)
    ('thunder',   3, 37.0, -3.10, 48.18, 2.0, 2.0),   # 叶片3雷击 (1m斑)
]

N_IMAGES = 600
OUT_DIR = '/home/developer/yolov5/datasets/turbine'
random.seed(42)


def set_camera_pose(x, y, z, yaw):
    """设置相机位姿 (进程内gz-transport, 避免CLI开销)"""
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_pb2 import Pose as GzPose
    from gz.msgs10.boolean_pb2 import Boolean
    global _gz_ctx
    if '_gz_ctx' not in globals():
        _gz_ctx = {'node': GzNode()}

    req = GzPose()
    req.name = 'turbine_cam'
    req.position.x = x
    req.position.y = y
    req.position.z = z
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    req.orientation.z = sy
    req.orientation.w = cy
    resp = Boolean()
    _gz_ctx['node'].request('/world/default/set_pose', req, GzPose, Boolean, 3000)
    return resp.data


def capture_frame():
    """从ROS2话题抓一帧 (单次rclpy初始化, 复用全局节点)"""
    import rclpy
    from sensor_msgs.msg import Image as RosImage
    from cv_bridge import CvBridge
    global _rclpy_ctx
    if '_rclpy_ctx' not in globals():
        rclpy.init()
        _rclpy_ctx = {
            'node': rclpy.create_node('turbine_capture'),
            'bridge': CvBridge(),
            'got': [False], 'frame': [None],
        }
        def cb(msg):
            if not _rclpy_ctx['got'][0]:
                _rclpy_ctx['frame'][0] = _rclpy_ctx['bridge'].imgmsg_to_cv2(msg, 'passthrough')
                _rclpy_ctx['got'][0] = True
        _rclpy_ctx['node'].create_subscription(RosImage, '/drone/camera/image_raw', cb, 10)

    _rclpy_ctx['got'][0] = False
    _rclpy_ctx['frame'][0] = None
    node = _rclpy_ctx['node']
    # 抓2帧: 第1帧可能是旧位置画面, 第2帧才是新位置渲染
    t0 = time.time()
    frame_seq = 0
    while frame_seq < 2 and time.time() - t0 < 5:
        rclpy.spin_once(node, timeout_sec=0.05)
        if _rclpy_ctx['got'][0]:
            frame_seq += 1
            _rclpy_ctx['got'][0] = False
    return _rclpy_ctx['frame'][0]


def project_defect(cam_pos, cam_yaw, defect):
    """世界坐标缺陷 → 像素坐标 + 可见性"""
    dx, dy, dz = defect[2] - cam_pos[0], defect[3] - cam_pos[1], defect[4] - cam_pos[2]
    # 相机坐标系: 相机yaw旋转
    cos_y, sin_y = math.cos(-cam_yaw), math.sin(-cam_yaw)
    x_cam = dx * cos_y - dy * sin_y
    y_cam = dx * sin_y + dy * cos_y
    z_cam = dz
    if x_cam <= 0.5:  # 在相机后面
        return None
    # 针孔投影
    u = FX * y_cam / x_cam + CX
    v = -FY * z_cam / x_cam + CY
    if u < 0 or u >= IMG_W or v < 0 or v >= IMG_H:
        return None
    # 缺陷近似像素尺寸 (收紧30%使框贴合暗色核心)
    sx = defect[5] / x_cam * FX * 0.7
    sy = defect[6] / x_cam * FY * 0.7
    if sx < 8 or sy < 8:
        return None
    return (u, v, sx, sy)


def main():
    os.makedirs(os.path.join(OUT_DIR, 'images', 'train'), exist_ok=True)
    os.makedirs(os.path.join(OUT_DIR, 'labels', 'train'), exist_ok=True)
    os.makedirs(os.path.join(OUT_DIR, 'images', 'val'), exist_ok=True)
    os.makedirs(os.path.join(OUT_DIR, 'labels', 'val'), exist_ok=True)

    valid = 0
    # 每个缺陷轮流作为瞄准目标, 从随机距离/偏移拍摄
    per_defect = N_IMAGES // len(DEFECTS)
    for i in range(N_IMAGES):
        d = DEFECTS[i % len(DEFECTS)]
        dx, dy, dz = d[2], d[3], d[4]

        # 相机位置: 距离缺陷4-10m (近距离, 缺陷占画面大)
        dist = random.uniform(4, 10)
        az = random.uniform(0, 2 * math.pi)
        el = random.uniform(-0.6, 0.6)
        cam_x = dx + dist * math.cos(el) * math.cos(az)
        cam_y = dy + dist * math.cos(el) * math.sin(az)
        cam_z = dz + dist * math.sin(el)
        if cam_z < 2:
            cam_z = 2.0

        # yaw瞄准缺陷
        yaw = math.atan2(dy - cam_y, dx - cam_x)

        set_camera_pose(cam_x, cam_y, cam_z, yaw)
        time.sleep(1.2)  # 等待Gazebo渲染新位姿画面
        img = capture_frame()
        if img is None:
            print(f'[{i}] 抓帧失败, 跳过', flush=True)
            continue

        # 投影所有缺陷
        labels = []
        for d2 in DEFECTS:
            proj = project_defect((cam_x, cam_y, cam_z), yaw, d2)
            if proj is None:
                continue
            u, v, sx, sy = proj
            x1 = (u - sx / 2) / IMG_W
            y1 = (v - sy / 2) / IMG_H
            x2 = (u + sx / 2) / IMG_W
            y2 = (v + sy / 2) / IMG_H
            # 检查该区域是否有缺陷特征: 缺陷是深色/彩色标记, 框内必须有暗像素
            px1, py1 = max(0, int(u - sx / 2)), max(0, int(v - sy / 2))
            px2, py2 = min(IMG_W, int(u + sx / 2)), min(IMG_H, int(v + sy / 2))
            region = img[py1:py2, px1:px2]
            if region.size == 0:
                continue
            if region.min() > 120:
                continue  # 全是亮背景, 缺陷不可见(遮挡或角度不对)
            # YOLO格式: class cx cy w h (中心点+宽高)
            cx_n = max(0.0, min(1.0, (x1 + x2) / 2))
            cy_n = max(0.0, min(1.0, (y1 + y2) / 2))
            w_n = min(1.0, x2 - x1)
            h_n = min(1.0, y2 - y1)
            labels.append(f'{d2[1]} {cx_n:.6f} {cy_n:.6f} {w_n:.6f} {h_n:.6f}')

        if not labels:
            continue

        split = 'val' if valid % 10 == 0 else 'train'
        idx = valid
        valid += 1
        cv2.imwrite(os.path.join(OUT_DIR, 'images', split, f'{idx:05d}.jpg'), img)
        with open(os.path.join(OUT_DIR, 'labels', split, f'{idx:05d}.txt'), 'w') as f:
            f.write('\n'.join(labels))

        if valid % 20 == 0:
            print(f'已采集 {valid} 张有效图 (尝试{i+1}/{N_IMAGES})', flush=True)

    print(f'\n完成: {valid} 张有效标注图')
    # 写数据集配置
    yaml = f"""path: {OUT_DIR}
train: images/train
val: images/val

names:
  0: craze
  1: corrosion
  2: surface_injure
  3: thunderstrike
  4: crack
  5: hide_craze

nc: 6
"""
    with open('/home/developer/yolov5/data/turbine.yaml', 'w') as f:
        f.write(yaml)
    print('数据集配置: data/turbine.yaml')


if __name__ == '__main__':
    main()
