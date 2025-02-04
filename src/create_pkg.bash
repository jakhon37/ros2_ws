
ros2 pkg create --build-type ament_python motor_control --dependencies rclpy geometry_msgs
ros2 pkg create --build-type ament_python mpu6050_imu --dependencies rclpy sensor_msgs
pip3 install smbus2
