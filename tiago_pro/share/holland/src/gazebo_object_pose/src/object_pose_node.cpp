#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

class ObjectPoseNode : public rclcpp::Node {
public:
  ObjectPoseNode()
  : Node("object_pose_node") {
    client_ = this->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state");
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/gazebo_object_marker", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&ObjectPoseNode::timer_callback, this));
  }

private:
  void timer_callback() {
    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "/gazebo/get_entity_state not available");
      return;
    }

    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = "wooden_peg_board";       // Change to your object name
    request->reference_frame = "base_footprint"; // Ask Gazebo directly in robot base frame

    client_->async_send_request(request,
    [this](rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture resp){
      auto res = resp.get();

      if (!res->success) {
        RCLCPP_WARN(this->get_logger(), "Failed to get entity state");
        return;
      }

      // Publish marker for RViz
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_footprint";
      marker.header.stamp = this->now();
      marker.ns = "gazebo_object";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = res->state.pose; // Pose is already in base_footprint
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.1;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      marker_pub_->publish(marker);

      // Optional: log pose for MoveIt planning
      RCLCPP_INFO(this->get_logger(),
                  "Object in base_footprint: pos(%.3f, %.3f, %.3f), "
                  "quat(%.3f, %.3f, %.3f, %.3f)",
                  res->state.pose.position.x,
                  res->state.pose.position.y,
                  res->state.pose.position.z,
                  res->state.pose.orientation.x,
                  res->state.pose.orientation.y,
                  res->state.pose.orientation.z,
                  res->state.pose.orientation.w);
    });
  }

  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectPoseNode>());
  rclcpp::shutdown();
  return 0;
}
