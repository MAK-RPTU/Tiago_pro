import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from attach_gazebo_interfaces.srv import AttachCommand
from math import radians, sin, cos


class AttachServer(Node):
    def __init__(self):
        super().__init__('attach_gazebo')

        # Declare parameters with defaults
        self.declare_parameter('reference_frame', 'gripper_left_finger_link')
        self.declare_parameter('model_name', 'aruco_cube')
        self.declare_parameter('x_offset', 0.044)
        self.declare_parameter('y_offset', 0.0)
        self.declare_parameter('z_offset', -0.2)
        self.declare_parameter('orientation_x', None)
        self.declare_parameter('orientation_y', None)
        self.declare_parameter('orientation_z', None)
        self.declare_parameter('orientation_w', None)

        # Get parameters
        self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.x_offset = self.get_parameter('x_offset').get_parameter_value().double_value
        self.y_offset = self.get_parameter('y_offset').get_parameter_value().double_value
        self.z_offset = self.get_parameter('z_offset').get_parameter_value().double_value

        # Orientation defaults to 90° about X if None provided
        ox = self.get_parameter('orientation_x').get_parameter_value().double_value
        oy = self.get_parameter('orientation_y').get_parameter_value().double_value
        oz = self.get_parameter('orientation_z').get_parameter_value().double_value
        ow = self.get_parameter('orientation_w').get_parameter_value().double_value

        if any(val is None for val in [ox, oy, oz, ow]):
            angle_rad = radians(90)
            self.orientation_x = sin(angle_rad / 2)
            self.orientation_y = 0.0
            self.orientation_z = 0.0
            self.orientation_w = cos(angle_rad / 2)
        else:
            self.orientation_x = ox
            self.orientation_y = oy
            self.orientation_z = oz
            self.orientation_w = ow

        # Gazebo service client
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')

        self.attach_timer = None
        self.service = self.create_service(AttachCommand, 'attach_control', self.handle_request)
        self.get_logger().info('Attach server is ready.')

    def handle_request(self, request, response):
        if request.command == 'close':
            self.get_logger().info('Attach request received.')
            self.attach_timer = self.create_timer(0.05, self.attach_object)
            response.success = True
        elif request.command == 'open':
            self.get_logger().info('Detach request received.')
            if self.attach_timer:
                self.attach_timer.cancel()
            response.success = True
        else:
            self.get_logger().error('Invalid command. Use \"open\" or \"close\".')
            response.success = False
        return response

    def attach_object(self):
        req = SetEntityState.Request()
        req.state.name = self.model_name
        req.state.reference_frame = self.reference_frame
        req.state.pose.position.x = self.x_offset
        req.state.pose.position.y = self.y_offset
        req.state.pose.position.z = self.z_offset

        req.state.pose.orientation.x = self.orientation_x
        req.state.pose.orientation.y = self.orientation_y
        req.state.pose.orientation.z = self.orientation_z
        req.state.pose.orientation.w = self.orientation_w

        self.client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = AttachServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()