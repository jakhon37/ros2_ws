#!/usr/bin/env python3
"""
autonomous_car_launch.py
Launches the motor control, MPU6050, and includes the RPLIDAR A1 launch file.
"""
# /home/ubuntu/ros2_ws/install/sllidar_ros2/share/sllidar_ros2/launch/sllidar_a1_lauch_headless.py
# /home/ubuntu/ros2_ws/src/sllidar_ros2/launch/sllidar_a1_lauch_headless.py
# /home/ubuntu/ros2_ws/install/sllidar_ros2/share/sllidar_ros2/launch/sllidar_a1_lauch_headless.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the sllidar_ros2 package's launch file
    sllidar_launch_path = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_a1_lauch_headless.py'
    )
    
    return LaunchDescription([
        # Motor Controller Node
        Node(
            package='motor_control',
            executable='motor_controller',
            name='motor_controller',
            output='screen'
        ),
        # MPU6050 IMU Node
        Node(
            package='mpu6050_imu',
            executable='mpu6050_node',
            name='mpu6050_node',
            output='screen'
        ),
        # Include the sllidar_ros2 A1 launch file
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0', #'/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
