#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gazebo_msgs/msg/contacts_state.hpp"

class Midterm : public rclcpp::Node
{
public:
  Midterm();

private:
  void on_contacts(const gazebo_msgs::msg::ContactsState::SharedPtr msg);

  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contacts_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  geometry_msgs::msg::Twist cmd_vel_msg_;
  bool bumper_pressed_;
  bool going_backward_;
  bool turning_left_;
  bool turning_right_;
  int turns_;
  int direction_;
  float linear_speed_;
  float angular_speed_;
};
