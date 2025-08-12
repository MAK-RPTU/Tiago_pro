#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time


class ScriptedMotion(Node):
    def __init__(self):
        super().__init__('teleop')
        self.publisher_ = self.create_publisher(TwistStamped, '/mobile_base_controller/cmd_vel', 10)

        # Declare parameters with defaults
        self.declare_parameter('linear_speed', 0.0)       # m/s
        self.declare_parameter('linear_distance', 0.0)    # m
        self.declare_parameter('angular_speed', 0.0)      # rad/s
        self.declare_parameter('angular_angle', 0.0)      # rad

        # Get parameter values
        self.linear_speed = float(self.get_parameter('linear_speed').get_parameter_value().double_value)
        self.linear_distance = float(self.get_parameter('linear_distance').get_parameter_value().double_value)
        self.angular_speed = float(self.get_parameter('angular_speed').get_parameter_value().double_value)
        self.angular_angle = float(self.get_parameter('angular_angle').get_parameter_value().double_value)

        self.get_logger().info(
            f"Starting motion: linear_speed={self.linear_speed}, "
            f"linear_distance={self.linear_distance}, "
            f"angular_speed={self.angular_speed}, "
            f"angular_angle={self.angular_angle}"
        )

        # Execute motions only if non-zero
        if self.linear_speed != 0.0 and self.linear_distance != 0.0:
            self.move_linear(self.linear_speed, self.linear_distance)

        if self.angular_speed != 0.0 and self.angular_angle != 0.0:
            self.rotate(self.angular_speed, self.angular_angle)

        self.get_logger().info('Motion complete.')

    def move_linear(self, speed, distance):
        # Determine motion direction from distance
        direction = 1.0 if distance >= 0 else -1.0
        speed = abs(speed) * direction

        duration = abs(distance / speed)  # This now works correctly with signs
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
