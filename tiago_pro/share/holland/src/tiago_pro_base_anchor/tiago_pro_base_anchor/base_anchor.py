import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState, GetEntityState
from attach_gazebo_interfaces.srv import AttachCommand


class BaseAnchorServer(Node):
    def __init__(self):
        super().__init__('base_anchor')

        # Declare parameters with defaults
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('model_name', 'tiago-pro')

        # Get parameters
        self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value

        # Gazebo service clients
        self.set_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.get_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')

        while not self.set_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')
        while not self.get_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/get_entity_state service...')

        # Service to attach/detach
        self.service = self.create_service(AttachCommand, 'base_anchor_control', self.handle_request)

        self.locked_pose = None
        self.anchor_timer = None
        self.get_logger().info('Base Anchor Server ready.')

    def handle_request(self, request, response):
        if request.command == 'hold':  # Attach
            self.get_logger().info('Attach request received.')
            # Get current pose once
            get_req = GetEntityState.Request()
            get_req.name = self.model_name
            get_req.reference_frame = self.reference_frame

            future = self.get_client.call_async(get_req)
            future.add_done_callback(self.store_current_pose)

            response.success = True

        elif request.command == 'release':  # Detach
            self.get_logger().info('Detach request received.')
            if self.anchor_timer:
                self.anchor_timer.cancel()
                self.anchor_timer = None
            self.locked_pose = None
            response.success = True

        else:
            self.get_logger().error('Invalid command. Use "hold" or "release".')
            response.success = False
        return response

    def store_current_pose(self, future):
        result = future.result()
        if result and result.success:
            self.locked_pose = result.state.pose
            self.get_logger().info(f'Locked pose stored: {self.locked_pose}')
            self.anchor_timer = self.create_timer(0.05, self.lock_base_pose)
        else:
            self.get_logger().error('Failed to get current pose')

    def lock_base_pose(self):
        if self.locked_pose is None:
            return
        req = SetEntityState.Request()
        req.state.name = self.model_name
        req.state.reference_frame = self.reference_frame
        req.state.pose = self.locked_pose
        self.set_client.call_async(req)  # async, non-blocking


def main(args=None):
    rclpy.init(args=args)
    node = BaseAnchorServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
