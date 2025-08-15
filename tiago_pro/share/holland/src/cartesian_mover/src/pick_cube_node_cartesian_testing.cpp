// CArtesian TEsting
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pick_cube_node");
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm_right");
    
    // Set the reference frame
    move_group.setPoseReferenceFrame("gripper_right_base_link");
    
    // Get current end-effector pose
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
    
    // Define the target pose (move -0.05m in X direction relative to gripper_grasping_frame)
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x -= 0.05;
    
    // Plan a Cartesian path
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;  // No jump threshold
    const double eef_step = 0.01;  // Step size for interpolation
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    if (fraction > 0.95) // Ensure successful planning
    {
        RCLCPP_INFO(node->get_logger(), "Cartesian path computed successfully (%.2f%% achieved)", fraction * 100.0);
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory_ = trajectory;
        move_group.execute(cartesian_plan);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Cartesian path planning failed (%.2f%% achieved)", fraction * 100.0);
    }
    
    rclcpp::shutdown();
    return 0;
}