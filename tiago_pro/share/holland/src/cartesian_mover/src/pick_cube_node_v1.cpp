#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

class TiagoPickPlace
{
public:
  explicit TiagoPickPlace(const rclcpp::Node::SharedPtr &node) : node_(node)
  {
    // MoveIt groups for Tiago Pro
    move_group_arm_right_torso_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_right_torso");
    move_group_arm_left_torso_  = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_left_torso");
    move_group_both_arms_torso_  = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "both_arms_torso");
    move_group_arm_right_       = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_right");
    move_group_arm_left_        = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm_left");
    move_group_gripper_right_   = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "gripper_right");
    move_group_gripper_left_   = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "gripper_left");

    // Common motion settings for both torso arms
    for (auto &group : {move_group_arm_right_torso_, move_group_arm_left_torso_, move_group_both_arms_torso_})
    {
      group->setMaxVelocityScalingFactor(0.5);
      group->setMaxAccelerationScalingFactor(0.5);
      group->setPlanningTime(10.0);
    }

    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  }

  // Add cube for collision avoidance
  void addCollisionBox()
  {
    moveit_msgs::msg::CollisionObject cube;
    cube.header.frame_id = move_group_arm_right_torso_->getPlanningFrame();
    cube.id = "table_cube";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {1.0, 0.8, 0.83}; // X, Y, Z in meters

    geometry_msgs::msg::Pose cube_pose;
    cube_pose.position.x = 1.127;
    cube_pose.position.y = -0.112;
    cube_pose.position.z = 0.83 / 2.0; // Half height
    cube_pose.orientation.w = 1.0;

    cube.primitives.push_back(primitive);
    cube.primitive_poses.push_back(cube_pose);
    cube.operation = cube.ADD;

    planning_scene_interface_->applyCollisionObject(cube);
    RCLCPP_INFO(node_->get_logger(), "Added collision cube to planning scene.");
  }

  // Move both arms to home
  void moveBothArmsHome()
  {
      std::vector<double> home_position;

      double torso_pos = 0.1;  // meters
      home_position.push_back(torso_pos);

      // Right arm joints in degrees → radians
      std::vector<double> right_arm_deg = {21.0, -105.0, 27.0, -135.0, 0.0, -69.0, 0.0};
      for (double deg : right_arm_deg)
          home_position.push_back(deg * M_PI / 180.0);

      // Left arm joints in degrees → radians
      std::vector<double> left_arm_deg = {-21.0, -105.0, -27.0, -135.0, 0.0, -69.0, 0.0};
      for (double deg : left_arm_deg)
          home_position.push_back(deg * M_PI / 180.0);

      move_group_both_arms_torso_->setJointValueTarget(home_position);
      executeMovement(*move_group_both_arms_torso_, "Moved both arms to home position.", "Failed to move both arms to home position.");
  }

  void moveToPregraspPositionLeftarmTorso()
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.424;
    target_pose.position.y = 0.6;
    target_pose.position.z = 1.0;
    target_pose.orientation.x = 0.563;
    target_pose.orientation.y = -0.385;
    target_pose.orientation.z = 0.623;
    target_pose.orientation.w = 0.383;

    move_group_arm_left_torso_->setPoseTarget(target_pose);
    executeMovement(*move_group_arm_left_torso_, "Moved right arm torso to pregrasp position successfully.", "Failed to move to pregrasp position.");
  }

  void moveToPregraspPositionRightarmTorso()
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.45;
    target_pose.position.y = -0.77;
    target_pose.position.z = 0.93;
    target_pose.orientation.x = 0.68;
    target_pose.orientation.y = 0.39;
    target_pose.orientation.z = 0.59;
    target_pose.orientation.w = -0.19;

    move_group_arm_right_torso_->setPoseTarget(target_pose);
    executeMovement(*move_group_arm_right_torso_, "Moved right arm torso to pregrasp position successfully.", "Failed to move to pregrasp position.");
  }

  // void moveTograspPositionRightarm()
  // {
  //   geometry_msgs::msg::Pose target_pose;
  //   target_pose.position.x = 0.370;
  //   target_pose.position.y = -0.043;
  //   target_pose.position.z = 1.147;
  //   target_pose.orientation.w = 1.0;

  //   move_group_arm_right_->setPoseTarget(target_pose);
  //   executeMovement(*move_group_arm_right_, "Moved to grasp position successfully.", "Failed to move to grasp position.");
  // }

  void moveTograspPositionLeftarm()
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.695;
    target_pose.position.y = 0.256;
    target_pose.position.z = 1.044;
    target_pose.orientation.x = 0.563;
    target_pose.orientation.y = -0.385;
    target_pose.orientation.z = 0.623;
    target_pose.orientation.w = 0.383;

    move_group_arm_left_->setPoseTarget(target_pose);
    executeMovement(*move_group_arm_left_, "Moved to grasp position successfully.", "Failed to move to grasp position.");
  }

  void moveTograspPositionRightarm()
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.535;
    target_pose.position.y = -0.388;
    target_pose.position.z = 1.035;
    target_pose.orientation.x = 0.649;
    target_pose.orientation.y = 0.332;
    target_pose.orientation.z = 0.631;
    target_pose.orientation.w = -0.264;

    move_group_arm_right_->setPoseTarget(target_pose);
    executeMovement(*move_group_arm_right_, "Moved to grasp position successfully.", "Failed to move to grasp position.");
  }

  void RightArmCartesian(double distance)
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

  void LeftArmCartesian(double distance)
  {
    geometry_msgs::msg::Pose start_pose = move_group_arm_left_->getCurrentPose().pose;
    std::vector<geometry_msgs::msg::Pose> waypoints = {start_pose};
    start_pose.position.z -= distance;
    waypoints.push_back(start_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_arm_left_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction == 1.0)
      executeTrajectory(*move_group_arm_left_, trajectory, "Approach executed successfully.", "Failed to execute approach.");
    else
      RCLCPP_ERROR(node_->get_logger(), "Failed to compute approach path.");
  }

  void controlGripper(const std::string &action)
  {
    double grip_position;
    
    if (action == "close")
      grip_position = 0.04;  // closed position
    else if (action == "open")
      grip_position = 0.0; // open position
    else
    {
      RCLCPP_WARN(node_->get_logger(), "Unknown gripper command: %s", action.c_str());
      return;
    }

    move_group_gripper_right_->setJointValueTarget({grip_position});
    executeMovement(*move_group_gripper_right_, action + " gripper!", "Failed to " + action + " gripper!");
  }

  

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_right_torso_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_left_torso_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_both_arms_torso_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_right_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_left_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_right_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_left_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  void executeMovement(moveit::planning_interface::MoveGroupInterface &group, const std::string &success_msg, const std::string &fail_msg)
  {
    group.setStartStateToCurrentState();
    if (group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      RCLCPP_INFO(node_->get_logger(), "%s", success_msg.c_str());
    else
      RCLCPP_ERROR(node_->get_logger(), "%s", fail_msg.c_str());
  }

  void executeTrajectory(moveit::planning_interface::MoveGroupInterface &group,
                       const moveit_msgs::msg::RobotTrajectory &trajectory,
                       const std::string &success_msg,
                       const std::string &fail_msg)
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

  app->addCollisionBox();

  // Move both arms to home
  // app->moveBothArmsHome();

  // app->moveToPregraspPositionLeftarmTorso();

  // app->moveToPregraspPositionRightarmTorso();

  // app->moveTograspPositionLeftarm();

  // app->moveTograspPositionRightarm();

  app->controlGripper("open");
  app->controlGripper("close");

  app->LeftArmCartesian(0.05);

  app->RightArmCartesian(0.05);

  // app->moveToPregraspPositionLeftarmTorso();

  // app->moveToPregraspPositionRightarmTorso();


  // app->moveToPregraspPositionLeftarmTorso();

  // app->moveBothArmsHome();
  // // Example Cartesian move for right arm
  // geometry_msgs::msg::Pose cart_pose;
  // cart_pose = app->getGroupByName("arm_right_torso")->getCurrentPose().pose;
  // cart_pose.position.z += 0.1;
  // app->cartesianMove("arm_right_torso", cart_pose);

  // // Example joint-space move for left arm
  // geometry_msgs::msg::Pose joint_pose;
  // joint_pose.orientation.w = 1.0;
  // joint_pose.position.x = 0.5;
  // joint_pose.position.y = 0.2;
  // joint_pose.position.z = 1.0;
  // app->jointSpaceMove("arm_left_torso", joint_pose);

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
