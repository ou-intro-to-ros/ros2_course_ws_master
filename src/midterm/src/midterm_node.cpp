#include <chrono>
#include "midterm/midterm_node.hpp"

Midterm::Midterm() : Node("midterm_node")
{
  contacts_sub_ = this->create_subscription<gazebo_msgs::msg::ContactsState>("/bumper_states", 10, std::bind(&Midterm::on_contacts, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  bumper_pressed_ = false;
  going_backward_ = false;
  turning_left_ = false;
  turning_right_ = false;
  turns_ = 0;
  direction_ = 1;
  linear_speed_ = 0.25;
  angular_speed_ = 1.5708;

  RCLCPP_INFO(this->get_logger(), "Midterm Bumper Node Started");
}

void Midterm::on_contacts(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
{
  bumper_pressed_ = !msg->states.empty();

  if (bumper_pressed_)
  {
    if (!going_backward_)
    {
      if (msg->states[0].collision1_name.find("left_bumper_link_collision") != std::string::npos || msg->states[0].collision2_name.find("left_bumper_link_collision") != std::string::npos)
      {
        going_backward_ = true;
        turning_right_ = true;
        direction_ = 1;
        if (turning_left_)
        {
          turning_left_ = false;
          turns_ = 0;
        }
        else
        {
          turns_++;
        }
        if (turns_ > 3)
        {
          RCLCPP_INFO(this->get_logger(), "Too many turns in one direction, switching...");
          turning_right_ = false;
          turning_left_ = true;
          turns_ = 0;
          direction_ = -1;
        }
        RCLCPP_INFO(this->get_logger(), "Left bumper pressed! Reversing and turning...");
        cmd_vel_msg_.linear.x = -linear_speed_;
        cmd_vel_msg_.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel_msg_);
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        cmd_vel_msg_.linear.x = 0.0;
        cmd_vel_msg_.angular.z = -angular_speed_ * direction_;
        cmd_vel_pub_->publish(cmd_vel_msg_);
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        going_backward_ = false;
      }
      else if (msg->states[0].collision1_name.find("right_bumper_link_collision") != std::string::npos || msg->states[0].collision2_name.find("right_bumper_link_collision") != std::string::npos)
      {
        // Right bumper pressed
        going_backward_ = true;
        turning_left_ = true;
        direction_ = 1;
        if (turning_right_)
        {
          turning_right_ = false;
          turns_ = 0;
        }
        else
        {
          turns_++;
        }
        if (turns_ > 3)
        {
          RCLCPP_INFO(this->get_logger(), "Too many turns in one direction, switching...");
          turning_left_ = false;
          turning_right_ = true;
          turns_ = 0;
          direction_ = -1;
        }
        RCLCPP_INFO(this->get_logger(), "Right bumper pressed! Moving backwards and turning...");
        cmd_vel_msg_.linear.x = -linear_speed_;
        cmd_vel_msg_.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel_msg_);
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        cmd_vel_msg_.linear.x = 0.0;
        cmd_vel_msg_.angular.z = angular_speed_ * direction_;
        cmd_vel_pub_->publish(cmd_vel_msg_);
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        going_backward_ = false;
      }
    }
  }
  else
  {
    // If bumper is not pressed, move forward
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Moving forward...");
    cmd_vel_msg_.linear.x = linear_speed_;
    cmd_vel_msg_.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel_msg_);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Midterm>());
  rclcpp::shutdown();
  return 0;
}