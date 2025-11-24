#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <map>

class NavigationController : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigationController() : Node("navigation_controller_node")
  {
    aruco_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/detected_aruco_pose", 10,
      std::bind(&NavigationController::arucoCallback, this, std::placeholders::_1));

    command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/navigation_command", 10,
      std::bind(&NavigationController::commandCallback, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

    RCLCPP_INFO(this->get_logger(), "Navigation Controller Node initialized");
  }

private:
  void arucoCallback(const geometry_msgs::msg::PoseStamped::SharedPtr /*msg*/)
  {
    // Store ArUco positions for later navigation
  }

  void commandCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received command: %s", msg->data.c_str());
    
    if (msg->data.find("goto_aruco") != std::string::npos)
    {
      // Parse ArUco ID and navigate
    }
  }

  void navigateToAruco(int aruco_id)
  {
    auto it = aruco_positions_.find(aruco_id);
    if (it == aruco_positions_.end())
    {
      RCLCPP_WARN(this->get_logger(), "ArUco ID %d not found in database", aruco_id);
      return;
    }

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    goal.pose = it->second;

    sendGoal(goal);
  }

  void sendGoal(const geometry_msgs::msg::PoseStamped& goal)
  {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(this->get_logger(), "Navigation action server not available");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NavigationController::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.result_callback =
      std::bind(&NavigationController::resultCallback, this, std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "Sending navigation goal");
    nav_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goalResponseCallback(const GoalHandleNav::SharedPtr& goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted");
    }
  }

  void resultCallback(const GoalHandleNav::WrappedResult& result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Navigation aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Navigation canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  std::map<int, geometry_msgs::msg::Pose> aruco_positions_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}