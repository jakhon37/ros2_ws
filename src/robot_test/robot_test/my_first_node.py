#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node 


class MyNode(Node):
    def __init__(self):
        super().__init__('first_node')
        self.counter_ = 0
        # self.get_logger().info('ROS2 is running...')
        self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        self.get_logger().info(f"ROS2 is running . ...{self.counter_}")
        self.counter_+=1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__=="__main__":
    main()
    
    