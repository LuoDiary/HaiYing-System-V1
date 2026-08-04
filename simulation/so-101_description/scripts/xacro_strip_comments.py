#!/usr/bin/env python3
"""展开 xacro 并移除 XML 注释，兼容 Humble gazebo_ros2_control 参数解析器."""
import re
import sys

import xacro
from ament_index_python.packages import get_package_share_directory


def main():
    if len(sys.argv) != 2:
        raise SystemExit('usage: xacro_strip_comments.py <robot.urdf.xacro>')
    xml = xacro.process_file(sys.argv[1]).toxml()
    xml = re.sub(r'<!--.*?-->', '', xml, flags=re.DOTALL)
    # Gazebo Classic 不解析 package:// URI；仅 Gazebo 包装层使用绝对文件 URI，主 URDF 保持不变。
    share_dir = get_package_share_directory('so-101_description')
    xml = xml.replace('package://so-101_description/', f'file://{share_dir}/')
    print(xml)


if __name__ == '__main__':
    main()
