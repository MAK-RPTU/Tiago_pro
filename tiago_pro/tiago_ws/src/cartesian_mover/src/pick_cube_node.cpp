#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

class TiagoPickPlace
{
public:
  explicit TiagoPickPlace(const rclcpp::Node::SharedPtr &node) : node_(node)
  {
    move_group_arm_right_torso_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_right_torso");
    move_group_arm_right_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_right");
    move_group_gripper_right_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "gripper_right");

    move_group_arm_right_torso_->setMaxVelocityScalingFactor(0.5);
    move_group_arm_right_torso_->setMaxAccelerationScalingFactor(0.5);
    move_group_arm_right_torso_->setPlanningTime(10.0);
  }

  void moveToHomePosition()
  {
    std::vector<double> home_position = {0.34, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    move_group_arm_right_torso_->setJointValueTarget(home_position);
    executeMovement(*move_group_arm_right_torso_, "Moved to home position successfully.", "Failed to move to home position.");
  }

  void moveToPregraspPosition()
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.497;
    target_pose.position.y = -0.564;
    target_pose.position.z = 1.127;
    target_pose.orientation.w = 1.0;

    move_group_arm_right_torso_->setPoseTarget(target_pose);
    executeMovement(*move_group_arm_right_torso_, "Moved to pregrasp position successfully.", "Failed to move to pregrasp position.");
  }

  void safe_position(const std::vector<double> &joint_angles_input)
  {
    if (joint_angles_input.size() != 8)
    {
      RCLCPP_ERROR(node_->get_logger(), "Input vector must have exactly 8 elements (1 torso + 7 arm joints).");
      return;
    }

    std::vector<double> target_joint_values;
    target_joint_values.push_back(joint_angles_input[0]);

    for (size_t i = 1; i < joint_angles_input.size(); ++i)
    {
      double angle_rad = joint_angles_input[i] * (M_PI / 180.0);
      target_joint_values.push_back(angle_rad);
    }

    move_group_arm_right_torso_->setJointValueTarget(target_joint_values);
    executeMovement(*move_group_arm_right_torso_, "Moved to safe position successfully.", "Failed to move to safe position.");
  }

  void moveTograspPosition()
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.370;
    target_pose.position.y = -0.043;
    target_pose.position.z = 1.147;
    target_pose.orientation.w = 1.0;

    move_group_arm_right_torso_->setPoseTarget(target_pose);
    executeMovement(*move_group_arm_right_torso_, "Moved to grasp position successfully.", "Failed to move to grasp position.");
  }

  void rotate_ee(double wrist_angle)
  {
    std::vector<double> joint_values = move_group_arm_right_torso_->getCurrentJointValues();
    if (!joint_values.empty())
    {
      joint_values.back() = wrist_angle;
      move_group_arm_right_torso_->setJointValueTarget(joint_values);
      executeMovement(*move_group_arm_right_torso_, "Rotated end-effector successfully.", "Failed to rotate end-effector.");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Joint state vector is empty!");
    }
  }

  void approachObject(double distance)
  {
    geometry_msgs::msg::Pose start_pose = move_group_arm_right_->getCurrentPose().pose;
    std::vector<geometry_msgs::msg::Pose> waypoints = {start_pose};
    start_pose.position.z -= distance;
    waypoints.push_back(start_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_arm_right_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction == 1.0)
      executeTrajectory(*move_group_arm_right_, trajectory, "Approach executed successfully.", "Failed to execute approach.");
    else
      RCLCPP_ERROR(node_->get_logger(), "Failed to compute approach path.");
  }

  void retreatObject(double distance)
  {
    geometry_msgs::msg::Pose start_pose = move_group_arm_right_torso_->getCurrentPose().pose;
    std::vector<geometry_msgs::msg::Pose> waypoints = {start_pose};
    start_pose.position.z += distance;
    waypoints.push_back(start_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_arm_right_torso_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    if (fraction == 1.0)
      executeTrajectory(*move_group_arm_right_torso_, trajectory, "Retreat executed successfully.", "Failed to execute retreat.");
    else
      RCLCPP_ERROR(node_->get_logger(), "Failed to compute retreat path.");
  }

  void controlGripper(const std::string &action)
  {
    std::vector<double> grip_positions;
    if (action == "close")
      grip_positions = {0.023, 0.023};
    else if (action == "open")
      grip_positions = {0.043, 0.043};
    else
    {
      RCLCPP_WARN(node_->get_logger(), "Unknown gripper command: %s", action.c_str());
      return;
    }

    move_group_gripper_right_->setJointValueTarget(grip_positions);
    executeMovement(*move_group_gripper_right_, action + " gripper!", "Failed to " + action + " gripper!");
  }

  void moveToPlacePosition()
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.541;
    target_pose.position.y = -0.260;
    target_pose.position.z = 1.13;
    target_pose.orientation.w = 1.0;

    move_group_arm_right_torso_->setPoseTarget(target_pose);
    executeMovement(*move_group_arm_right_torso_, "Moved to place position successfully.", "Failed to move to place position.");
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_right_torso_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_right_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_right_;

  void executeMovement(moveit::planning_interface::MoveGroupInterface &group, const std::string &success_msg, const std::string &fail_msg)
  {
    group.setStartStateToCurrentState();
    if (group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      RCLCPP_INFO(node_->get_logger(), "%s", success_msg.c_str());
    else
      RCLCPP_ERROR(node_->get_logger(), "%s", fail_msg.c_str());
  }

  void executeTrajectory(moveit::planning_interface::MoveGroupInterface &group, const moveit_msgs::msg::RobotTrajectory &trajectory, const std::string &success_msg, const std::string &fail_msg)
  {
    group.setStartStateToCurrentState();
    if (group.execute(trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      RCLCPP_INFO(node_->get_logger(), "%s", success_msg.c_str());
    else
      RCLCPP_ERROR(node_->get_logger(), "%s", fail_msg.c_str());
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tiago_pick_place");
  auto app = std::make_shared<TiagoPickPlace>(node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  app->moveToHomePosition();
  app->moveToPregraspPosition();

  std::vector<double> joints1 = {0.34, 22.0, 8.0, -122.0, 88.0, 84.0, -70.0, -54.0};
  app->safe_position(joints1);

  std::vector<double> joints2 = {0.213, 12.0, 26.0, -104.0, 102.0, 75.0, -64.0, -71.0};
  app->safe_position(joints2);

  app->controlGripper("open");
  app->approachObject(0.08);
  app->retreatObject(0.08);
  app->moveToPregraspPosition();
  app->moveToPlacePosition();
  app->rotate_ee(-M_PI);
  app->approachObject(0.05);
  app->controlGripper("open");

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
