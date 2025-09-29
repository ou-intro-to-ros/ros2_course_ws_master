#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

geometry_msgs::msg::PointStamped previous_point;
void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("turtle_sub");
  
  auto sub_point = node->create_subscription<geometry_msgs::msg::PointStamped>("turtle1/point", 1, pointCallback);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  if (previous_point.header.stamp.sec != 0) {
    double distance = std::sqrt(std::pow(msg->point.x - previous_point.point.x, 2) + std::pow(msg->point.y - previous_point.point.y, 2) + std::pow(msg->point.z - previous_point.point.z, 2));
    double time_diff = (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9) - (previous_point.header.stamp.sec + previous_point.header.stamp.nanosec * 1e-9);
    double speed = distance / time_diff;
    previous_point = *msg;
    RCLCPP_INFO(rclcpp::get_logger("turtle_sub"), "Speed: %lf", speed);
  } else {
    previous_point = *msg;
    RCLCPP_INFO(rclcpp::get_logger("turtle_sub"), "Received first point.");
  }
}