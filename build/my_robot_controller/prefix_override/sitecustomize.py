import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jakhon37/myprojects/mmm/autonomus/auto_ros/ros2_tutor/ros2_ws/install/my_robot_controller'
