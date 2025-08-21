#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <attach_gazebo_interfaces/srv/attach_command.hpp>
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

  // Move both arms to nav pose
  void moveBothArmsNavPose()
  {
      std::vector<double> nav_position;

      double torso_pos = 0.1;  // meters
      nav_position.push_back(torso_pos);

      // Right arm joints in degrees → radians
      std::vector<double> right_arm_deg = {21.0, -105.0, 27.0, -135.0, 0.0, 0.0, 0.0};
      for (double deg : right_arm_deg)
          nav_position.push_back(deg * M_PI / 180.0);

      // Left arm joints in degrees → radians
      std::vector<double> left_arm_deg = {-21.0, -105.0, -27.0, -135.0, 0.0, 0.0, 0.0};
      for (double deg : left_arm_deg)
          nav_position.push_back(deg * M_PI / 180.0);

      move_group_both_arms_torso_->setJointValueTarget(nav_position);
      executeMovement(*move_group_both_arms_torso_, "Moved both arms to home position.", "Failed to move both arms to home position.");
  }

  // void moveToPregraspPositionLeftarmTorso()
  // {
  //   geometry_msgs::msg::Pose target_pose;
  //   target_pose.position.x = 0.424;
  //   target_pose.position.y = 0.6;
  //   target_pose.position.z = 1.0;
  //   target_pose.orientation.x = 0.563;
  //   target_pose.orientation.y = -0.385;
  //   target_pose.orientation.z = 0.623;
  //   target_pose.orientation.w = 0.383;

  //   move_group_arm_left_torso_->setPoseTarget(target_pose);
  //   executeMovement(*move_group_arm_left_torso_, "Moved right arm torso to pregrasp position successfully.", "Failed to move to pregrasp position.");
  // }

  // void moveToPregraspPositionRightarmTorso()
  // {
  //   geometry_msgs::msg::Pose target_pose;
  //   target_pose.position.x = 0.45;
  //   target_pose.position.y = -0.77;
  //   target_pose.position.z = 0.93;
  //   target_pose.orientation.x = 0.68;
  //   target_pose.orientation.y = 0.39;
  //   target_pose.orientation.z = 0.59;
  //   target_pose.orientation.w = -0.19;

  //   move_group_arm_right_torso_->setPoseTarget(target_pose);
  //   executeMovement(*move_group_arm_right_torso_, "Moved right arm torso to pregrasp position successfully.", "Failed to move to pregrasp position.");
  // }

  void moveToPregraspPositionLeftarmTorso()
  {
      std::vector<double> pregrasp_position;

      // double torso_pos = 0.3;  // meters
      // pregrasp_position.push_back(torso_pos);

      // Right arm joints in degrees → radians
      std::vector<double> left_arm_deg = {90.0, -89.0, -24, -79.0, 0.0, -57.0, -90.0};
      for (double deg : left_arm_deg)
          pregrasp_position.push_back(deg * M_PI / 180.0);

      move_group_arm_left_->setJointValueTarget(pregrasp_position);
      executeMovement(*move_group_arm_left_, "Moved right torso arms to pregrasp position.", "Failed to move both arms to home position.");
  }

  void moveToPregraspPositionRightarmTorso()
  {
      std::vector<double> pregrasp_position;

      double torso_pos = 0.3;  // meters
      pregrasp_position.push_back(torso_pos);

      // Right arm joints in degrees → radians
      std::vector<double> right_arm_deg = {-90.0, -89.0, 24, -79.0, 0.0, -57.0, 90.0};
      for (double deg : right_arm_deg)
          pregrasp_position.push_back(deg * M_PI / 180.0);

      move_group_arm_right_torso_->setJointValueTarget(pregrasp_position);
      executeMovement(*move_group_arm_right_torso_, "Moved right torso arms to pregrasp position.", "Failed to move both arms to home position.");
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

  // void moveTograspPositionLeftarm()
  // {
  //   geometry_msgs::msg::Pose target_pose;
  //   target_pose.position.x = 0.695;
  //   target_pose.position.y = 0.256;
  //   target_pose.position.z = 1.044;
  //   target_pose.orientation.x = 0.563;
  //   target_pose.orientation.y = -0.385;
  //   target_pose.orientation.z = 0.623;
  //   target_pose.orientation.w = 0.383;

  //   move_group_arm_left_->setPoseTarget(target_pose);
  //   executeMovement(*move_group_arm_left_, "Moved to grasp position successfully.", "Failed to move to grasp position.");
  // }

  void moveTograspPositionLeftarm()
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.625;
    target_pose.position.y = 0.209;
    target_pose.position.z = 1.074;
    target_pose.orientation.x = 0.709;
    target_pose.orientation.y = -0.144;
    target_pose.orientation.z = 0.666;
    target_pose.orientation.w = 0.185;

    move_group_arm_left_->setPoseTarget(target_pose);
    executeMovement(*move_group_arm_left_, "Moved to grasp position successfully.", "Failed to move to grasp position.");
  }

  void moveTograspPositionRightarm()
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.486;
    target_pose.position.y = -0.343;
    target_pose.position.z = 1.082;
    target_pose.orientation.x = 0.710;
    target_pose.orientation.y = 0.148;
    target_pose.orientation.z = 0.665;
    target_pose.orientation.w = -0.181;

    move_group_arm_right_->setPoseTarget(target_pose);
    executeMovement(*move_group_arm_right_, "Moved to grasp position successfully.", "Failed to move to grasp position.");
  }

  // void moveTograspPositionRightarm()
  // {
  //   geometry_msgs::msg::Pose target_pose;
  //   target_pose.position.x = 0.535;
  //   target_pose.position.y = -0.388;
  //   target_pose.position.z = 1.035;
  //   target_pose.orientation.x = 0.649;
  //   target_pose.orientation.y = 0.332;
  //   target_pose.orientation.z = 0.631;
  //   target_pose.orientation.w = -0.264;

  //   move_group_arm_right_->setPoseTarget(target_pose);
  //   executeMovement(*move_group_arm_right_, "Moved to grasp position successfully.", "Failed to move to grasp position.");
  // }

  void RightArmCartesian(const std::string &axis, double distance)
  {
      move_group_arm_right_->setStartStateToCurrentState();

      // Set the reference frame
      move_group_arm_right_->setPoseReferenceFrame("gripper_right_base_link");

      // Get current pose
      geometry_msgs::msg::Pose current_pose = move_group_arm_right_->getCurrentPose().pose;

      // Define target pose
      geometry_msgs::msg::Pose target_pose = current_pose;

      // Apply displacement on chosen axis
      if (axis == "x")
          target_pose.position.x += distance;
      else if (axis == "y")
          target_pose.position.y += distance;
      else if (axis == "z")
          target_pose.position.z += distance;
      else
      {
          RCLCPP_ERROR(node_->get_logger(), "Invalid axis '%s'. Use 'x', 'y', or 'z'.", axis.c_str());
          return;
      }

      // Prepare waypoints
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(target_pose);

      // Plan Cartesian path
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      double fraction = move_group_arm_right_->computeCartesianPath(
          waypoints, eef_step, jump_threshold, trajectory);

      if (fraction > 0.95)  // Ensure successful planning
      {
          RCLCPP_INFO(node_->get_logger(),
                      "Right arm Cartesian path computed successfully (%.2f%% achieved)",
                      fraction * 100.0);

          // Create plan object and assign trajectory
          moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
          cartesian_plan.trajectory_ = trajectory;

          // Execute plan
          if (move_group_arm_right_->execute(cartesian_plan) ==
              moveit::planning_interface::MoveItErrorCode::SUCCESS)
          {
              RCLCPP_INFO(node_->get_logger(), "Right arm Cartesian path executed successfully.");
          }
          else
          {
              RCLCPP_ERROR(node_->get_logger(), "Failed to execute right arm Cartesian path.");
          }
      }
      else
      {
          RCLCPP_ERROR(node_->get_logger(),
                      "Right arm Cartesian path planning failed (%.2f%% achieved)",
                      fraction * 100.0);
      }
  }


  void LeftArmCartesian(const std::string &axis, double distance)
  {
      move_group_arm_left_->setStartStateToCurrentState();

      // Set the reference frame
      move_group_arm_left_->setPoseReferenceFrame("gripper_left_base_link");

      // Get current pose
      geometry_msgs::msg::Pose current_pose = move_group_arm_left_->getCurrentPose().pose;

      // Define target pose
      geometry_msgs::msg::Pose target_pose = current_pose;

      if (axis == "x")
          target_pose.position.x += distance;
      else if (axis == "y")
          target_pose.position.y += distance;
      else if (axis == "z")
          target_pose.position.z += distance;
      else
      {
          RCLCPP_ERROR(node_->get_logger(), "Invalid axis '%s'. Use 'x', 'y', or 'z'.", axis.c_str());
          return;
      }

      // Prepare waypoints
      std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};

      // Plan Cartesian path
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      double fraction = move_group_arm_left_->computeCartesianPath(
          waypoints, eef_step, jump_threshold, trajectory);

      if (fraction > 0.95)
      {
          RCLCPP_INFO(node_->get_logger(),
                      "Left arm Cartesian path computed successfully (%.2f%% achieved)",
                      fraction * 100.0);

          moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
          cartesian_plan.trajectory_ = trajectory;

          if (move_group_arm_left_->execute(cartesian_plan) ==
              moveit::planning_interface::MoveItErrorCode::SUCCESS)
          {
              RCLCPP_INFO(node_->get_logger(), "Left arm Cartesian path executed successfully.");
          }
          else
          {
              RCLCPP_ERROR(node_->get_logger(), "Failed to execute left arm Cartesian path.");
          }
      }
      else
      {
          RCLCPP_ERROR(node_->get_logger(),
                      "Left arm Cartesian path planning failed (%.2f%% achieved)",
                      fraction * 100.0);
      }
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


  void callAttachService(const std::string &service_name, const std::string &command)
  {
      // Create a temporary node for service call
      auto temp_node = rclcpp::Node::make_shared("attach_client_node");
      auto client = temp_node->create_client<attach_gazebo_interfaces::srv::AttachCommand>(service_name);

      // Wait for service
      if (!client->wait_for_service(std::chrono::seconds(5))) {
          RCLCPP_ERROR(node_->get_logger(), "Service %s not available", service_name.c_str());
          return;
      }

      auto request = std::make_shared<attach_gazebo_interfaces::srv::AttachCommand::Request>();
      request->command = command;

      // Send request
      auto future = client->async_send_request(request);

      // Use spin_until_future_complete on the temp node
      if (rclcpp::spin_until_future_complete(temp_node, future) ==
          rclcpp::FutureReturnCode::SUCCESS)
      {
          auto response = future.get();
          if (response->success)
              RCLCPP_INFO(node_->get_logger(), "Service %s executed with command: %s",
                          service_name.c_str(), command.c_str());
          else
              RCLCPP_WARN(node_->get_logger(), "Service %s failed to execute command: %s",
                          service_name.c_str(), command.c_str());
      }
      else
      {
          RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s", service_name.c_str());
      }
  }

  void callBaseAnchorService(const std::string &command)
  {
      auto temp_node = rclcpp::Node::make_shared("base_anchor_client");
      auto client = temp_node->create_client<attach_gazebo_interfaces::srv::AttachCommand>("/base_anchor_control");

      if (!client->wait_for_service(std::chrono::seconds(5))) {
          RCLCPP_ERROR(node_->get_logger(), "Base anchor service not available");
          return;
      }

      auto request = std::make_shared<attach_gazebo_interfaces::srv::AttachCommand::Request>();
      request->command = command;

      auto future = client->async_send_request(request);

      if (rclcpp::spin_until_future_complete(temp_node, future) ==
          rclcpp::FutureReturnCode::SUCCESS)
      {
          auto response = future.get();
          if (response->success)
              RCLCPP_INFO(node_->get_logger(), "Base anchor %s succeeded", command.c_str());
          else
              RCLCPP_WARN(node_->get_logger(), "Base anchor %s failed", command.c_str());
      }
      else {
          RCLCPP_ERROR(node_->get_logger(), "Failed to call base anchor service");
      }
  }

  void runTeleop(double speed, double distance)
  {
      std::stringstream cmd;
      cmd << "ros2 run teleop_tiago teleop --ros-args "
          << "-p linear_speed:=" << speed
          << " -p linear_distance:=" << distance;

      RCLCPP_INFO(node_->get_logger(), "Executing teleop: %s", cmd.str().c_str());

      int ret = std::system(cmd.str().c_str());

      if (ret == 0)
          RCLCPP_INFO(node_->get_logger(), "Teleop motion complete.");
      else
          RCLCPP_ERROR(node_->get_logger(), "Teleop failed with code %d", ret);
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

  app->moveToPregraspPositionLeftarmTorso();
  app->moveToPregraspPositionRightarmTorso();
  app->moveTograspPositionLeftarm();
  app->moveTograspPositionRightarm();

//   app->controlGripper("open");
//   app->controlGripper("close");

  // app->moveBothArmsHome();

  app->LeftArmCartesian("x", -0.08);
  app->callAttachService("/attach_control_left", "close");
  app->LeftArmCartesian("x", +0.2);

  // Release base before moving
  app->callBaseAnchorService("release");
  // Move base with teleop
  app->runTeleop(0.2, -0.2);
  app->runTeleop(0.2, +0.2);
  // Hold base again after motion
  app->callBaseAnchorService("hold");

  // app->moveBothArmsNavPose();
  app->LeftArmCartesian("y", -0.329);
  app->LeftArmCartesian("z", -0.12);
  app->LeftArmCartesian("x", -0.3);
  app->callAttachService("/attach_control_left", "open");
  app->LeftArmCartesian("x", +0.2);
  app->LeftArmCartesian("z", -0.1);



  // app->RightArmCartesian("z", -0.1);
  app->RightArmCartesian("x", -0.08);
  app->callAttachService("/attach_control_right", "close");
  app->RightArmCartesian("x", +0.2);
  app->RightArmCartesian("y", +0.265);
  app->RightArmCartesian("x", -0.32);
  app->callAttachService("/attach_control_right", "open");
  app->RightArmCartesian("x", +0.2);
  app->RightArmCartesian("z", -0.246);

  // app->RightArmCartesian("x", -0.2);

  // app->moveTograspPositionLeftarm();
  // app->moveTograspPositionRightarm();


  // app->moveToPregraspPositionLeftarmTorso();
  // app->moveToPregraspPositionRightarmTorso();


  // app->moveToPregraspPositionLeftarmTorso();

  app->moveBothArmsHome();
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
