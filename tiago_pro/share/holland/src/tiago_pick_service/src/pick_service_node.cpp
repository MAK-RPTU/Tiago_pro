#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "tiago_pick_service/srv/pick_pose.hpp"
#include <geometry_msgs/msg/pose.hpp>

class PickServiceNode : public rclcpp::Node
{
public:
  PickServiceNode() : Node("pick_service_node")
  {
    using namespace std::placeholders;
    srv_ = create_service<tiago_pick_service::srv::PickPose>(
      "pick_pose", std::bind(&PickServiceNode::handle_pick, this, _1, _2));
    
    auto moveit_node = rclcpp::Node::make_shared("moveit_interface_node");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node, "arm_right_torso");
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node, "gripper_right");
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
  }

private:
  void handle_pick(const std::shared_ptr<tiago_pick_service::srv::PickPose::Request> request,
                   std::shared_ptr<tiago_pick_service::srv::PickPose::Response> response)
  {
    move_group_->setPoseTarget(request->pose);
    bool success = (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
      std::vector<double> gripper_target = {0.023, 0.023};
      gripper_group_->setJointValueTarget(gripper_target); 
      gripper_group_->move();
    }

    response->success = success;
  }

  rclcpp::Service<tiago_pick_service::srv::PickPose>::SharedPtr srv_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickServiceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
