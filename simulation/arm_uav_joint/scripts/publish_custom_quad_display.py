#!/usr/bin/env python3
"""发布 V7 custom quad 的 Gazebo SDF。"""
import os
import sys
from pathlib import Path
from xml.etree import ElementTree

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


DISPLAY_TOPIC = '/custom_quad_333_display_description'
USE_PX4_PLUGINS_ENV = 'SO101_CUSTOM_QUAD_USE_PX4_PLUGINS'
STATIC_MODEL_ENV = 'SO101_CUSTOM_QUAD_STATIC'

ARM_JOINT_NAMES = {
    'J1_Rotation',
    'J2_Shoulder_Pitch',
    'J3_Elbow_Pitch',
    'J4_Wrist_Pitch',
    'J5_Wrist_Roll',
}


def env_flag(name: str) -> bool:
    """读取布尔环境变量。"""
    return os.environ.get(name, '').strip().lower() in {'1', 'true', 'yes', 'on'}


def env_flag_default(name: str, default: bool) -> bool:
    """读取可配置布尔环境变量，并在未设置时使用默认值。"""
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def build_display_sdf(source_path: Path) -> str:
    """清理 ros2_control 冲突，并按环境选择是否加载 PX4 飞行器插件。"""
    root = ElementTree.fromstring(source_path.read_text(encoding='utf-8'))
    model = root.find('model')
    if model is None:
        raise ValueError(f'SDF 缺少 model 节点: {source_path}')

    use_px4_plugins = env_flag(USE_PX4_PLUGINS_ENV)
    static_model = env_flag_default(STATIC_MODEL_ENV, True)
    for element in list(model):
        if element.tag == 'plugin':
            filename = element.get('filename', '')
            # custom quad 内嵌的控制器路径属于另一套工作空间，不能与独立 SO-101 重复加载。
            if 'gazebo_ros2_control' in filename or not use_px4_plugins:
                model.remove(element)
            continue
        if element.tag == 'include' and not use_px4_plugins:
            model.remove(element)
            continue
        # GPS include 被移除后，配套关节也必须移除，否则 Gazebo 会报告断开的姿态图。
        if (
            element.tag == 'joint'
            and element.get('name') == 'gps0_joint'
            and not use_px4_plugins
        ):
            model.remove(element)
            continue
        # 当前飞行验证使用固定姿态的桥接机械臂；其碰撞、质量和惯量保留在组合体中，
        # 让 PX4 在真实挂载负载下完成姿态估计与起飞。后续关节控制接入时再恢复转动轴。
        if (
            element.tag == 'joint'
            and element.get('name') in ARM_JOINT_NAMES
            and not static_model
        ):
            element.set('type', 'fixed')
            for child in list(element):
                if child.tag in {'axis', 'axis2'}:
                    element.remove(child)

    # 原始 SDF 有两个 static 节点，统一成一个，保证 Gazebo 解析结果确定。
    for static in model.findall('static'):
        model.remove(static)
    static = ElementTree.SubElement(model, 'static')
    static.text = '1' if static_model else '0'

    return ElementTree.tostring(root, encoding='unicode')


class CustomQuadDisplayPublisher(Node):
    """以 transient-local 话题保存展示 SDF，供 spawn_entity 读取。"""

    def __init__(self, sdf: str) -> None:
        super().__init__('custom_quad_display_description')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, DISPLAY_TOPIC, qos)
        message = String()
        message.data = sdf
        self.publisher.publish(message)
        mode = '静态' if env_flag_default(STATIC_MODEL_ENV, True) else '动态'
        self.get_logger().info(f'已发布 custom quad {mode} SDF（桥接机械臂参与动力学）')


def main(args: list[str]) -> int:
    rclpy.init(args=args)
    package_share = Path(get_package_share_directory('arm_uav_joint'))
    source_path = package_share / 'models' / 'custom_quad_333' / 'custom_quad_333.sdf'
    node = CustomQuadDisplayPublisher(build_display_sdf(source_path))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
