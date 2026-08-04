from setuptools import find_packages, setup


PACKAGE_NAME = "haiying_zhixun_bridge"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("tests",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README.md"]),
        (f"share/{PACKAGE_NAME}/config", ["config/arm_bridge.yaml"]),
        (f"share/{PACKAGE_NAME}/launch", ["launch/moveit_real_gui.launch.py"]),
        (
            f"share/{PACKAGE_NAME}/docs",
            [
                "docs/CAO_YUANYUAN_WORK.md",
                "docs/MEASUREMENT_CHECKLIST.md",
                "docs/LOCAL_IK_TO_REAL.md",
                "docs/MOVEIT_REAL_GUI.md",
                "docs/ROS2_INTEGRATION.md",
            ],
        ),
    ],
    install_requires=["PyYAML>=5.4"],
    zip_safe=True,
    maintainer="曹圆圆",
    maintainer_email="cao-yuanyuan@example.com",
    description="海鹰智巡项目机械臂仿真与实机桥接包",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "haiying-arm-bridge-node = haiying_zhixun_bridge.ros_node:main",
            "haiying-moveit-real-gui = haiying_zhixun_bridge.moveit_real_gui:main",
            "haiying-moveit-real-server = haiying_zhixun_bridge.moveit_real_server:main",
            "haiying-moveit-real-smoke = haiying_zhixun_bridge.moveit_real_smoke:main",
        ]
    },
)
