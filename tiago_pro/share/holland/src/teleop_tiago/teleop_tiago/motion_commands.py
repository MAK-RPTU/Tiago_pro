#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time

class ScriptedMotion(Node):
    def __init__(self):
        super().__init__('scripted_motion')
        self.publisher_ = self.create_publisher(TwistStamped, '/mobile_base_controller/cmd_vel', 10)
        self.get_logger().info('Starting scripted motion...')

        # Below you can define a sequence of motions

        # Move forward 2m
        self.move_linear(speed=0.2, distance=2.0)
        # Move backward 1m
        self.move_linear(speed=-0.2, distance=1.0)
        # Turn right (clockwise) 90 degrees
        self.rotate(angular_speed=-0.5, angle_rad=1.57)
        # Turn left (counter-clockwise) 90 degrees
        self.rotate(angular_speed=0.5, angle_rad=1.57)
        
        self.get_logger().info('Motion complete.')

    def move_linear(self, speed, distance):
        duration = abs(distance / speed)
        start_time = self.get_clock().now().nanoseconds / 1e9

        while self.get_clock().now().nanoseconds / 1e9 < start_time + duration:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = speed
            self.publisher_.publish(msg)
            time.sleep(0.1)

        self.stop_robot()

    def rotate(self, angular_speed, angle_rad):
        duration = abs(angle_rad / angular_speed)
        start_time = self.get_clock().now().nanoseconds / 1e9

        while self.get_clock().now().nanoseconds / 1e9 < start_time + duration:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.angular.z = angular_speed
            self.publisher_.publish(msg)
            time.sleep(0.1)

        self.stop_robot()

    def stop_robot(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = ScriptedMotion()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
