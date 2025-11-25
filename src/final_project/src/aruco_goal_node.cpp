// aruco_goal_node.cpp
// Navigate to a specific ArUco marker using saved positions

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cmath>

class ArucoGoalNode : public rclcpp::Node
{
public:
    ArucoGoalNode() : Node("aruco_goal_node")
    {
        this->declare_parameter<int>("target_marker_id", -1);
        this->declare_parameter<std::string>("markers_file", "");
        this->declare_parameter<double>("initial_pose_x", 0.25);
        this->declare_parameter<double>("initial_pose_y", 0.5);
        this->declare_parameter<double>("initial_pose_yaw", 0.0);
        
        this->get_parameter("target_marker_id", target_marker_id_);
        this->get_parameter("markers_file", markers_file_);
        this->get_parameter("initial_pose_x", initial_x_);
        this->get_parameter("initial_pose_y", initial_y_);
        this->get_parameter("initial_pose_yaw", initial_yaw_);
        
        if (markers_file_.empty()) {
            std::string pkg_dir = ament_index_cpp::get_package_share_directory("final_project");
            markers_file_ = pkg_dir + "/maps/aruco_markers.yaml";
        }
        
        // Publisher for initial pose
        initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);
        
        // Action client for Nav2
        nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");
        
        // Timer to set initial pose and navigate
        wait_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ArucoGoalNode::mainLoop, this));
        
        RCLCPP_INFO(get_logger(), "ArUco Goal Node initialized");
        RCLCPP_INFO(get_logger(), "Target marker ID: %d", target_marker_id_);
        RCLCPP_INFO(get_logger(), "Markers file: %s", markers_file_.c_str());
    }

private:
    void mainLoop()
    {
        // Step 1: Publish initial pose (do this 3 times)
        if (initial_pose_count_ < 3) {
            publishInitialPose();
            initial_pose_count_++;
            return;
        }
        
        // Step 2: Check if goal already sent
        if (goal_sent_) {
            return;
        }
        
        // Step 3: Wait for Nav2 to be ready
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Waiting for Nav2 action server...");
            return;
        }
        
        // Step 4: Load markers and navigate
        if (loadMarkers()) {
            navigateToMarker();
            goal_sent_ = true;
            wait_timer_->cancel();
        }
    }
    
    void publishInitialPose()
    {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.frame_id = "map";
        msg.header.stamp = now();
        
        msg.pose.pose.position.x = initial_x_;
        msg.pose.pose.position.y = initial_y_;
        msg.pose.pose.position.z = 0.0;
        
        // Convert yaw to quaternion
        msg.pose.pose.orientation.z = std::sin(initial_yaw_ / 2.0);
        msg.pose.pose.orientation.w = std::cos(initial_yaw_ / 2.0);
        
        // Set covariance
        msg.pose.covariance[0] = 0.25;   // x variance
        msg.pose.covariance[7] = 0.25;   // y variance
        msg.pose.covariance[35] = 0.06853891909122467;  // yaw variance
        
        initial_pose_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Published initial pose: (%.2f, %.2f, %.2f)", 
            initial_x_, initial_y_, initial_yaw_);
    }
    
    bool loadMarkers()
    {
        try {
            YAML::Node config = YAML::LoadFile(markers_file_);
            
            if (!config["markers"]) {
                RCLCPP_ERROR(get_logger(), "No 'markers' key found in YAML file");
                return false;
            }
            
            for (const auto& marker : config["markers"]) {
                int id = marker["id"].as<int>();
                double x = marker["x"].as<double>();
                double y = marker["y"].as<double>();
                markers_[id] = {x, y};
                RCLCPP_INFO(get_logger(), "Loaded marker %d at (%.2f, %.2f)", id, x, y);
            }
            
            RCLCPP_INFO(get_logger(), "Loaded %zu markers from file", markers_.size());
            return true;
            
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to load markers file: %s", e.what());
            return false;
        }
    }
    
    void navigateToMarker()
    {
        if (target_marker_id_ < 0) {
            RCLCPP_ERROR(get_logger(), "Invalid marker ID: %d", target_marker_id_);
            return;
        }
        
        if (markers_.find(target_marker_id_) == markers_.end()) {
            RCLCPP_ERROR(get_logger(), "Marker %d not found in saved positions!", target_marker_id_);
            RCLCPP_INFO(get_logger(), "Available markers:");
            for (const auto& [id, pos] : markers_) {
                RCLCPP_INFO(get_logger(), "  - Marker %d", id);
            }
            return;
        }
        
        auto [x, y] = markers_[target_marker_id_];
        
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = now();
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.w = 1.0;
        
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = goal;
        
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        
        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(get_logger(), "✓ Successfully reached marker %d!", target_marker_id_);
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(get_logger(), "✗ Navigation aborted");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(get_logger(), "Navigation canceled");
                        break;
                    default:
                        RCLCPP_ERROR(get_logger(), "Navigation failed");
                        break;
                }
            };
        
        send_goal_options.goal_response_callback =
            [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(get_logger(), "Goal rejected by Nav2");
                } else {
                    RCLCPP_INFO(get_logger(), "Goal accepted, navigating to marker %d...", target_marker_id_);
                }
            };
        
        RCLCPP_INFO(get_logger(), "Sending navigation goal to marker %d at (%.2f, %.2f)", 
            target_marker_id_, x, y);
        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr wait_timer_;
    
    std::map<int, std::pair<double, double>> markers_;
    std::string markers_file_;
    int target_marker_id_;
    double initial_x_, initial_y_, initial_yaw_;
    int initial_pose_count_ = 0;
    bool goal_sent_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoGoalNode>());
    rclcpp::shutdown();
    return 0;
}