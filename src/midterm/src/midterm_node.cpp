#include <chrono>
#include "midterm/midterm_node.hpp"

Midterm::Midterm() : Node("midterm_node")
{
  contacts_sub_ = this->create_subscription<gazebo_msgs::msg::ContactsState>("/bumper_states", 10, std::bind(&Midterm::on_contacts, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  side_timer_ = this->create_wall_timer(std::chrono::milliseconds(3000), std::bind(&Midterm::check_side_timer_callback, this));

  bumper_pressed_ = false;
  going_backward_ = false;
  going_sideways_ = false;
  linear_speed_ = 0.5;
  angular_speed_ = 1.5708;

  RCLCPP_INFO(this->get_logger(), "Midterm Bumper Node Started");
}

void Midterm::check_side_timer_callback()
{
  if (!going_backward_ && !going_sideways_)
  {
    going_sideways_ = true;
    RCLCPP_INFO(this->get_logger(), "Checking side wall...");
    cmd_vel_msg_.linear.x = 0.0;
    cmd_vel_msg_.angular.z = angular_speed_;
    cmd_vel_pub_->publish(cmd_vel_msg_);
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    cmd_vel_msg_.linear.x = linear_speed_;
    cmd_vel_msg_.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel_msg_);
    going_sideways_ = false;
  }
}

void Midterm::on_contacts(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
{
  bumper_pressed_ = !msg->states.empty();

  if (bumper_pressed_ && !going_sideways_ && !going_backward_)
  {
    going_backward_ = true;
    RCLCPP_INFO(this->get_logger(), "Bumper pressed! Reversing and turning...");
    cmd_vel_msg_.linear.x = -linear_speed_;
    cmd_vel_msg_.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel_msg_);
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    cmd_vel_msg_.linear.x = 0.0;
    cmd_vel_msg_.angular.z = -angular_speed_;
    cmd_vel_pub_->publish(cmd_vel_msg_);
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    cmd_vel_msg_.linear.x = linear_speed_;
    cmd_vel_msg_.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel_msg_);
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    going_backward_ = false;
    side_timer_->reset();
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Midterm>());
  rclcpp::shutdown();
  return 0;
}