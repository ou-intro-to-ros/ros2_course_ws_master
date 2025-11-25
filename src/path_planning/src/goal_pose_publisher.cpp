#include "path_planning/goal_pose_publisher.hpp"

GoalPosePublisher::GoalPosePublisher() : Node("goal_pose_publisher")
{
  publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

  init_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "/publish_goal", 10, std::bind(&GoalPosePublisher::initial_pose_callback, this, std::placeholders::_1));

  amcl_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10, std::bind(&GoalPosePublisher::amcl_pose_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "waiting for initial pose to be sent");
}

void GoalPosePublisher::initial_pose_callback(const std_msgs::msg::String::SharedPtr msg)
{
  (void)msg;

  if (!timer_) {
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&GoalPosePublisher::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "started publishing goal pose at 1Hz");
  }
}

void GoalPosePublisher::timer_callback()
{
  auto message = geometry_msgs::msg::PoseStamped();

  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "map";

  message.pose.position.x = 2.0;
  message.pose.position.y = 0.5;

  message.pose.orientation.w = 1.0;

  publisher_->publish(message);
  RCLCPP_INFO(this->get_logger(), "goal pose sent");
}

void GoalPosePublisher::amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!timer_) {
    return;
  }

  double dx = msg->pose.pose.position.x - 2.0;
  double dy = msg->pose.pose.position.y - 0.5;
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance < 0.5) {
    timer_->cancel();
    timer_.reset();
    RCLCPP_INFO(this->get_logger(), "goal reached, stopped publishing");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
