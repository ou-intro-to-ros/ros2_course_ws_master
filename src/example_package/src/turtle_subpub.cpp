#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_point;
void poseCallback(const turtlesim::msg::Pose::SharedPtr msg);

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("turtle_subpub");

  auto sub_pose = node->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 1, poseCallback);
  pub_point = node->create_publisher<geometry_msgs::msg::PointStamped>("turtle1/point", 1);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

void poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
{
  geometry_msgs::msg::PointStamped point;
  point.header.stamp = rclcpp::Clock().now();
  point.header.frame_id = "turtle1";
  point.point.x = msg->x;
  point.point.y = msg->y;
  point.point.z = 0.0;
  pub_point->publish(point);
}