import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/ros2_ws/src/robot_test/install/robot_test'
