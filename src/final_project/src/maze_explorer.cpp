// maze_explorer.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <limits>

class MazeExplorer : public rclcpp::Node
{
public:
    MazeExplorer() : Node("maze_explorer")
    {
        this->declare_parameter<std::string>("follow_side", "right");
        this->declare_parameter<double>("target_wall_distance", 0.25);
        this->declare_parameter<double>("forward_speed", 0.15);
        this->declare_parameter<double>("angular_speed", 0.8);
        
        this->get_parameter("follow_side", follow_side_);
        this->get_parameter("target_wall_distance", target_wall_distance_);
        this->get_parameter("forward_speed", forward_speed_);
        this->get_parameter("angular_speed", angular_speed_);
        
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MazeExplorer::scanCallback, this, std::placeholders::_1));
        
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MazeExplorer::mapCallback, this, std::placeholders::_1));
        
        camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&MazeExplorer::cameraCallback, this, std::placeholders::_1));
        
        aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        aruco_params_ = cv::aruco::DetectorParameters::create();
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&MazeExplorer::controlLoop, this));
        
        RCLCPP_INFO(get_logger(), "Wall-Following Maze Explorer initialized");
    }

private:
    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(cv_ptr->image, aruco_dict_, corners, ids, aruco_params_);
            
            for (size_t i = 0; i < ids.size(); i++) {
                if (detected_markers_.find(ids[i]) == detected_markers_.end()) {
                    try {
                        auto tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
                        detected_markers_[ids[i]] = {tf.transform.translation.x, tf.transform.translation.y};
                        RCLCPP_INFO(get_logger(), "ArUco %d at (%.2f, %.2f)", ids[i], 
                            detected_markers_[ids[i]].first, detected_markers_[ids[i]].second);
                    } catch (...) {}
                }
            }
        } catch (...) {}
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        current_scan_ = msg;
    }
    
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = msg;
        if (++map_update_count_ % 50 == 0) {
            checkExplorationProgress();
        }
    }
    
    void controlLoop() {
        /* YOUR EXISTING controlLoop CODE - KEEP AS IS */
        if (!current_scan_) return;
        if (exploration_complete_) {
            stopRobot();
            return;
        }
        if (in_recovery_) {
            executeRecovery();
            return;
        }
        
        int num_rays = static_cast<int>(current_scan_->ranges.size());
        if (num_rays <= 0) return;
        
        int front_margin = num_rays / 12;
        float front_dist = std::min(
            getMinDistance(0, front_margin),
            getMinDistance(num_rays - front_margin, num_rays)
        );
        
        int right_start = num_rays * 3 / 4;
        int right_end = num_rays * 7 / 8;
        float right_dist = getMinDistance(right_start, right_end);
        
        int left_start = num_rays / 8;
        int left_end = num_rays / 4;
        float left_dist = getMinDistance(left_start, left_end);
        
        const float INF = std::numeric_limits<float>::infinity();
        if (!std::isfinite(front_dist)) front_dist = INF;
        if (!std::isfinite(right_dist)) right_dist = INF;
        if (!std::isfinite(left_dist)) left_dist = INF;
        
        geometry_msgs::msg::Twist cmd;
        const float obstacle_threshold = 0.4f;
        const float decel_threshold = 0.6f;
        
        if (front_dist < obstacle_threshold) {
            cmd.linear.x = 0.0;
            cmd.angular.z = (follow_side_ == "right") ? angular_speed_ * 0.7 : -angular_speed_ * 0.7;
            state_ = "TURNING";
            stuck_counter_++;
            if (stuck_counter_ > 25) {
                startRecovery(follow_side_ == "right" ? "left" : "right");
                executeRecovery();
                return;
            }
        } else if (front_dist < decel_threshold) {
            float speed_factor = (front_dist - obstacle_threshold) / (decel_threshold - obstacle_threshold);
            cmd.linear.x = forward_speed_ * speed_factor * 0.5;
            cmd.angular.z = (follow_side_ == "right") ? angular_speed_ * 0.5 : -angular_speed_ * 0.5;
            state_ = "SLOWING";
            stuck_counter_ = 0;
        } else {
            stuck_counter_ = 0;
            
            if (follow_side_ == "right") {
                float error = right_dist - target_wall_distance_;
                if (right_dist > 1.5f) {
                    cmd.linear.x = forward_speed_ * 0.5;
                    cmd.angular.z = -angular_speed_ * 0.4;
                    state_ = "SEARCH_WALL";
                } else {
                    cmd.linear.x = forward_speed_;
                    cmd.angular.z = -1.2 * error;
                    cmd.angular.z = std::max(-angular_speed_ * 0.8f, std::min(angular_speed_ * 0.8f, cmd.angular.z));
                    state_ = (std::abs(error) < 0.05) ? "FOLLOWING" : (error > 0 ? "TOO_FAR" : "TOO_CLOSE");
                }
            } else {
                float error = left_dist - target_wall_distance_;
                if (left_dist > 1.5f) {
                    cmd.linear.x = forward_speed_ * 0.5;
                    cmd.angular.z = angular_speed_ * 0.4;
                    state_ = "SEARCH_WALL";
                } else {
                    cmd.linear.x = forward_speed_;
                    cmd.angular.z = 1.2 * error;
                    cmd.angular.z = std::max(-angular_speed_ * 0.8f, std::min(angular_speed_ * 0.8f, cmd.angular.z));
                    state_ = (std::abs(error) < 0.05) ? "FOLLOWING" : (error > 0 ? "TOO_FAR" : "TOO_CLOSE");
                }
            }
        }
        
        static float prev_linear_x = 0.0, prev_angular_z = 0.0;
        cmd.linear.x = 0.7 * cmd.linear.x + 0.3 * prev_linear_x;
        cmd.angular.z = 0.6 * cmd.angular.z + 0.4 * prev_angular_z;
        prev_linear_x = cmd.linear.x;
        prev_angular_z = cmd.angular.z;
        
        cmd_vel_pub_->publish(cmd);
    }
    
    void startRecovery(const std::string &rotate_direction) {
        in_recovery_ = true;
        recovery_phase_ = 0;
        recovery_phase_end_time_ = get_clock()->now() + rclcpp::Duration::from_seconds(0.7);
        recovery_rotate_direction_ = rotate_direction;
        stuck_counter_ = 0;
    }
    
    void executeRecovery() {
        auto now = get_clock()->now();
        geometry_msgs::msg::Twist cmd;
        if (recovery_phase_ == 0) {
            cmd.linear.x = -0.08;
            cmd.angular.z = 0.0;
            if (now >= recovery_phase_end_time_) {
                recovery_phase_ = 1;
                recovery_phase_end_time_ = now + rclcpp::Duration::from_seconds(0.7);
            }
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = (recovery_rotate_direction_ == "left") ? angular_speed_ : -angular_speed_;
            if (now >= recovery_phase_end_time_) {
                in_recovery_ = false;
                recovery_phase_ = 0;
            }
        }
        cmd_vel_pub_->publish(cmd);
    }
    
    float getMinDistance(int start_idx, int end_idx) {
        if (!current_scan_) return std::numeric_limits<float>::infinity();
        int n = static_cast<int>(current_scan_->ranges.size());
        start_idx = std::max(0, std::min(n-1, start_idx));
        end_idx = std::max(start_idx+1, std::min(n, end_idx));
        float min_dist = std::numeric_limits<float>::infinity();
        for (int i = start_idx; i < end_idx; i++) {
            float range = current_scan_->ranges[i];
            if (std::isfinite(range) && range > current_scan_->range_min && range < current_scan_->range_max) {
                min_dist = std::min(min_dist, range);
            }
        }
        return min_dist;
    }
    
    void checkExplorationProgress() {
        if (!current_map_) return;
        int unknown_count = 0;
        for (int8_t cell : current_map_->data) {
            if (cell == -1) unknown_count++;
        }
        double explored_ratio = 1.0 - (static_cast<double>(unknown_count) / current_map_->data.size());
        RCLCPP_INFO(get_logger(), "Exploration: %.1f%% | State: %s", explored_ratio * 100.0, state_.c_str());
        
        if (explored_ratio >= 0.92 && !exploration_complete_) {
            saveMarkersToFile();
            exploration_complete_ = true;
            stopRobot();
        }
    }
    
    void saveMarkersToFile() {
        std::string pkg_dir = ament_index_cpp::get_package_share_directory("final_project");
        std::string filepath = pkg_dir + "/maps/aruco_markers.yaml";
        std::ofstream file(filepath);
        file << "markers:\n";
        for (const auto& [id, pos] : detected_markers_) {
            file << "  - id: " << id << "\n    x: " << pos.first << "\n    y: " << pos.second << "\n";
        }
        file.close();
        RCLCPP_INFO(get_logger(), "Saved %zu markers to %s", detected_markers_.size(), filepath.c_str());
    }
    
    void stopRobot() {
        geometry_msgs::msg::Twist cmd;
        cmd_vel_pub_->publish(cmd);
    }
    
    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::map<int, std::pair<double, double>> detected_markers_;
    
    std::string follow_side_, state_ = "INIT", recovery_rotate_direction_ = "left";
    double target_wall_distance_, forward_speed_, angular_speed_;
    bool exploration_complete_ = false, in_recovery_ = false;
    int map_update_count_ = 0, stuck_counter_ = 0, recovery_phase_ = 0;
    rclcpp::Time recovery_phase_end_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MazeExplorer>());
    rclcpp::shutdown();
    return 0;
}
// // maze_explorer.cpp
// // Wall-following maze exploration with SLAM mapping

// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <cmath>
// #include <algorithm>
// #include <limits>
// #include <chrono>
// #include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/aruco.hpp>

// class MazeExplorer : public rclcpp::Node
// {
// public:
//     MazeExplorer() : Node("maze_explorer")
//     {
//         this->declare_parameter<std::string>("follow_side", "right");  // "right" or "left"
//         this->declare_parameter<double>("target_wall_distance", 0.25);
//         this->declare_parameter<double>("forward_speed", 0.15);
//         this->declare_parameter<double>("angular_speed", 0.8);
        
//         this->get_parameter("follow_side", follow_side_);
//         this->get_parameter("target_wall_distance", target_wall_distance_);
//         this->get_parameter("forward_speed", forward_speed_);
//         this->get_parameter("angular_speed", angular_speed_);
        
//         // Publishers
//         cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
//         // Subscribers
//         scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
//             "/scan", 10,
//             std::bind(&MazeExplorer::scanCallback, this, std::placeholders::_1));
        
//         map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
//             "/map", 10,
//             std::bind(&MazeExplorer::mapCallback, this, std::placeholders::_1));
        
//         // Control timer
//         control_timer_ = create_wall_timer(
//             std::chrono::milliseconds(100),  // 10Hz
//             std::bind(&MazeExplorer::controlLoop, this));
        
//         RCLCPP_INFO(get_logger(), "Wall-Following Maze Explorer initialized");
//         RCLCPP_INFO(get_logger(), "Following %s wall at %.2fm distance", 
//             follow_side_.c_str(), target_wall_distance_);
//     }

// private:
//     void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
//     {
//         current_scan_ = msg;
//     }
    
//     void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
//     {
//         current_map_ = msg;
        
//         // Check exploration progress every 50 map updates
//         if (++map_update_count_ % 50 == 0) {
//             checkExplorationProgress();
//         }
//     }
    
//     void controlLoop()
//     {
//         if (!current_scan_) return;
//         if (exploration_complete_) {
//             stopRobot();
//             return;
//         }
//         if (in_recovery_) {
//             executeRecovery();
//             return;
//         }
        
//         int num_rays = static_cast<int>(current_scan_->ranges.size());
//         if (num_rays <= 0) return;
        
//         // Front region (wrap around index 0)
//         int front_margin = num_rays / 12;
//         float front_dist = std::min(
//             getMinDistance(0, front_margin),
//             getMinDistance(num_rays - front_margin, num_rays)
//         );
        
//         // Right region
//         int right_start = num_rays * 3 / 4;
//         int right_end = num_rays * 7 / 8;
//         float right_dist = getMinDistance(right_start, right_end);
        
//         // Left region
//         int left_start = num_rays / 8;
//         int left_end = num_rays / 4;
//         float left_dist = getMinDistance(left_start, left_end);
        
//         const float INF = std::numeric_limits<float>::infinity();
//         if (!std::isfinite(front_dist)) front_dist = INF;
//         if (!std::isfinite(right_dist)) right_dist = INF;
//         if (!std::isfinite(left_dist)) left_dist = INF;
        
//         geometry_msgs::msg::Twist cmd;
        
//         const float obstacle_threshold = 0.4f;
//         const float decel_threshold = 0.6f;  // Start slowing down earlier
        
//         // Gradual deceleration when approaching obstacle
//         if (front_dist < obstacle_threshold) {
//             cmd.linear.x = 0.0;
//             cmd.angular.z = (follow_side_ == "right") ? angular_speed_ * 0.7 : -angular_speed_ * 0.7;
//             state_ = "TURNING";
//             stuck_counter_++;
//             if (stuck_counter_ > 25) {
//                 startRecovery(follow_side_ == "right" ? "left" : "right");
//                 executeRecovery();
//                 return;
//             }
//         } else if (front_dist < decel_threshold) {
//             // Deceleration zone - proportional to distance
//             float speed_factor = (front_dist - obstacle_threshold) / (decel_threshold - obstacle_threshold);
//             cmd.linear.x = forward_speed_ * speed_factor * 0.5;  // Scale down speed
//             cmd.angular.z = (follow_side_ == "right") ? angular_speed_ * 0.5 : -angular_speed_ * 0.5;
//             state_ = "SLOWING";
//             stuck_counter_ = 0;
//         } else {
//             stuck_counter_ = 0;
            
//             if (follow_side_ == "right") {
//                 float error = right_dist - target_wall_distance_;
                
//                 if (right_dist > 1.5f) {
//                     cmd.linear.x = forward_speed_ * 0.5;
//                     cmd.angular.z = -angular_speed_ * 0.4;
//                     state_ = "SEARCH_WALL";
//                 } else {
//                     cmd.linear.x = forward_speed_;
//                     cmd.angular.z = -1.2 * error;
//                     cmd.angular.z = std::max(-angular_speed_ * 0.8f, std::min(angular_speed_ * 0.8f, cmd.angular.z));
                    
//                     if (std::abs(error) < 0.05) {
//                         state_ = "FOLLOWING";
//                     } else if (error > 0) {
//                         state_ = "TOO_FAR";
//                     } else {
//                         state_ = "TOO_CLOSE";
//                     }
//                 }
//             } else {
//                 float error = left_dist - target_wall_distance_;
                
//                 if (left_dist > 1.5f) {
//                     cmd.linear.x = forward_speed_ * 0.5;
//                     cmd.angular.z = angular_speed_ * 0.4;
//                     state_ = "SEARCH_WALL";
//                 } else {
//                     cmd.linear.x = forward_speed_;
//                     cmd.angular.z = 1.2 * error;
//                     cmd.angular.z = std::max(-angular_speed_ * 0.8f, std::min(angular_speed_ * 0.8f, cmd.angular.z));
                    
//                     if (std::abs(error) < 0.05) {
//                         state_ = "FOLLOWING";
//                     } else if (error > 0) {
//                         state_ = "TOO_FAR";
//                     } else {
//                         state_ = "TOO_CLOSE";
//                     }
//                 }
//             }
//         }
        
//         // Smooth both linear and angular velocities
//         static float prev_linear_x = 0.0;
//         static float prev_angular_z = 0.0;
        
//         cmd.linear.x = 0.7 * cmd.linear.x + 0.3 * prev_linear_x;
//         cmd.angular.z = 0.6 * cmd.angular.z + 0.4 * prev_angular_z;
        
//         prev_linear_x = cmd.linear.x;
//         prev_angular_z = cmd.angular.z;
        
//         cmd_vel_pub_->publish(cmd);
//     }
    
//     // Recovery: simple two-phase backup then rotate away
//     void startRecovery(const std::string &rotate_direction)
//     {
//         in_recovery_ = true;
//         recovery_phase_ = 0;
//         // backup for 0.7s then rotate for 0.7s
//         rclcpp::Duration backup_dur = rclcpp::Duration::from_seconds(0.7);
//         rclcpp::Duration rotate_dur = rclcpp::Duration::from_seconds(0.7);
//         recovery_phase_end_time_ = get_clock()->now() + backup_dur;
//         recovery_rotate_direction_ = rotate_direction;
//         exploration_complete_ = false; // keep exploring after recovery
//         stuck_counter_ = 0;
//     }
    
//     void executeRecovery()
//     {
//         auto now = get_clock()->now();
//         geometry_msgs::msg::Twist cmd;
//         if (recovery_phase_ == 0) {
//             // backup
//             cmd.linear.x = -0.08;
//             cmd.angular.z = 0.0;
//             if (now >= recovery_phase_end_time_) {
//                 // move to rotate phase
//                 recovery_phase_ = 1;
//                 recovery_phase_end_time_ = now + rclcpp::Duration::from_seconds(0.7);
//             }
//         } else {
//             // rotate away
//             cmd.linear.x = 0.0;
//             if (recovery_rotate_direction_ == "left") {
//                 cmd.angular.z = angular_speed_;
//             } else {
//                 cmd.angular.z = -angular_speed_;
//             }
//             if (now >= recovery_phase_end_time_) {
//                 // finished recovery
//                 in_recovery_ = false;
//                 recovery_phase_ = 0;
//             }
//         }
//         cmd_vel_pub_->publish(cmd);
//     }
    
//     float getMinDistance(int start_idx, int end_idx)
//     {
//         if (!current_scan_) {
//             return std::numeric_limits<float>::infinity();
//         }
        
//         // clamp indices
//         int n = static_cast<int>(current_scan_->ranges.size());
//         start_idx = std::max(0, std::min(n-1, start_idx));
//         end_idx = std::max(start_idx+1, std::min(n, end_idx));
        
//         float min_dist = std::numeric_limits<float>::infinity();
        
//         for (int i = start_idx; i < end_idx; i++) {
//             float range = current_scan_->ranges[i];
//             if (std::isfinite(range) && range > current_scan_->range_min && range < current_scan_->range_max) {
//                 min_dist = std::min(min_dist, range);
//             }
//         }
        
//         return min_dist;
//     }
    
//     void checkExplorationProgress()
//     {
//         if (!current_map_) return;
        
//         int unknown_count = 0;
//         int total_count = current_map_->data.size();
        
//         for (int8_t cell : current_map_->data) {
//             if (cell == -1) unknown_count++;
//         }
        
//         double explored_ratio = 1.0 - (static_cast<double>(unknown_count) / total_count);
        
//         RCLCPP_INFO(get_logger(), "Exploration: %.1f%% | State: %s", 
//             explored_ratio * 100.0, state_.c_str());
        
//         // Consider complete at 92% (wall following won't reach 100%)
//         if (explored_ratio >= 0.92 && !exploration_complete_) {
//             RCLCPP_INFO(get_logger(), "Exploration complete! Stopping robot...");
//             exploration_complete_ = true;
//             stopRobot();
//         }
//     }
    
//     void stopRobot()
//     {
//         geometry_msgs::msg::Twist cmd;
//         cmd.linear.x = 0.0;
//         cmd.angular.z = 0.0;
//         cmd_vel_pub_->publish(cmd);
//     }
    
//     // Member variables
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
//     rclcpp::TimerBase::SharedPtr control_timer_;
    
//     sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
//     nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    
//     std::string follow_side_;
//     double target_wall_distance_;
//     double forward_speed_;
//     double angular_speed_;
    
//     std::string state_ = "INIT";
//     bool exploration_complete_ = false;
//     int map_update_count_ = 0;
    
//     // recovery / stuck handling
//     bool in_recovery_ = false;
//     int recovery_phase_ = 0;
//     rclcpp::Time recovery_phase_end_time_;
//     std::string recovery_rotate_direction_ = "left";
//     int stuck_counter_ = 0;
// };
 
// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<MazeExplorer>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }