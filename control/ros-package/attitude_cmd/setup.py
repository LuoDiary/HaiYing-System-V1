from setuptools import setup

package_name = 'attitude_cmd'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='ROS2 -> PX4 attitude control over TELEM serial + MAVLink',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'attitude_cmd_node = attitude_cmd.mavlink_link_node:main',
            'cmd_vel_to_attitude = attitude_cmd.cmd_vel_to_attitude:main',
            'hover_demo_node = attitude_cmd.hover_demo_node:main',
            'plot_vibration = attitude_cmd.plot_vibration:main',
            'plot_hover_drift = attitude_cmd.plot_hover_drift:main',
            'fake_px4 = attitude_cmd.fake_px4:main',
        ],
    },
)
