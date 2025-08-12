#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

geometry_msgs::msg::Pose get_wooden_peg_pose(rclcpp::Node::SharedPtr node)
{
  auto client = node->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state");
  if (!client->wait_for_service(2s))
    throw std::runtime_error("Gazebo service not available");

  auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
  request->name = "wooden_peg_board";
  request->reference_frame = "base_footprint";

  auto future = client->async_send_request(request);
  auto result = future.get();

  if (!result->success)
    throw std::runtime_error("Failed to get wooden peg pose");

  return result->state.pose;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "tiago_side_grasp",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface move_group(node, "arm_right_torso");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // --- Add cube in planning scene ---
  moveit_msgs::msg::CollisionObject cube;
  cube.header.frame_id = move_group.getPlanningFrame();
  cube.id = "table_cube";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {1.0, 0.08, 0.3}; // X, Y, Z
  geometry_msgs::msg::Pose cube_pose;
  cube_pose.position.x = 1.1466179082158041;
  cube_pose.position.y = 0.0996527785517715;
  cube_pose.position.z = 5.820053026192641e-05;
  cube_pose.orientation.x = -1.752262515208534e-07;
  cube_pose.orientation.y = 6.374598788105546e-07;
  cube_pose.orientation.z = -0.0005101454714367672;
  cube_pose.orientation.w = 0.999999869875572;
  cube.primitives.push_back(primitive);
  cube.primitive_poses.push_back(cube_pose);
  cube.operation = cube.ADD;
  planning_scene_interface.applyCollisionObject(cube);

  // --- Get wooden peg pose from Gazebo ---
  geometry_msgs::msg::Pose peg_pose;
  try {
    peg_pose = get_wooden_peg_pose(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
              "Peg pose wrt base_footprint: x=%.3f y=%.3f z=%.3f",
              peg_pose.position.x, peg_pose.position.y, peg_pose.position.z);

  // --- Apply Z-offset to avoid collision ---
  peg_pose.position.z += 0.1+0.15; // 10 cm above peg
  peg_pose.orientation.w = 1.0;

  // --- Apply Z offset to avoid collision ---
  double z_offset = 0.15; // 15 cm above
  peg_pose.position.z += z_offset;

  RCLCPP_INFO(node->get_logger(),
        "Target pose for grasp: x=%.3f y=%.3f z=%.3f",
        peg_pose.position.x, peg_pose.position.y, peg_pose.position.z);

  peg_pose.orientation.w = 1.0;
  peg_pose.position.x = 0.533;//0.34, 0.605, 0.533
  peg_pose.position.y = 0.292;//-0.198, -0.171, 0 .292
  peg_pose.position.z = 0.477;//0.565, 0.465, 0.477

  // --- Set as MoveIt target ---
  move_group.setPoseTarget(peg_pose);

  // --- Plan and execute ---
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_footprint",
      rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group.getRobotModel());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  visual_tools.prompt("Press 'Next' to plan");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    visual_tools.prompt("Press 'Next' to execute");
    move_group.execute(plan);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning failed!");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
