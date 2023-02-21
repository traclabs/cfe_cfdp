#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosidl_runtime_py import get_message_interfaces
from ament_index_python.packages import get_package_share_directory


class CFDPWrapper(Node):

    def __init__(self):
        super().__init__('cfdp_wrapper')

        self.get_logger().warn("================================================================")
        self.get_logger().warn("CFDPWrapper")
        self.get_logger().warn("================================================================")

        self._timer_period = 1.0  # seconds

        self._timer = self.create_timer(self._timer_period, self.timer_callback)


    def timer_callback(self):
        self.get_logger().warn("CFDPWrapper() -- tick")




def main(args=None):
    rclpy.init(args=args)

    wrapper = CFDPWrapper()
    rclpy.spin(wrapper)

    wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
