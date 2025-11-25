#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class MapSaverNode : public rclcpp::Node
{
public:
    MapSaverNode() : Node("map_saver_node")
    {
        this->declare_parameter<double>("exploration_complete_threshold", 0.92);
        this->get_parameter("exploration_complete_threshold", threshold_);
        
        // Get package maps directory
        std::string pkg_dir = ament_index_cpp::get_package_share_directory("final_project");
        map_save_path_ = pkg_dir + "/maps/explored_maze";
        
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapSaverNode::mapCallback, this, std::placeholders::_1));
        
        check_timer_ = create_wall_timer(
            std::chrono::seconds(5), std::bind(&MapSaverNode::checkExplorationStatus, this));
        
        RCLCPP_INFO(get_logger(), "Map Saver Node initialized");
        RCLCPP_INFO(get_logger(), "Will save map to: %s", map_save_path_.c_str());
        RCLCPP_INFO(get_logger(), "Threshold: %.1f%%", threshold_ * 100.0);
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { current_map_ = msg; }
    
    void checkExplorationStatus()
    {
        if (!current_map_ || map_saved_) return;
        
        int unknown_count = 0;
        for (int8_t cell : current_map_->data) {
            if (cell == -1) unknown_count++;
        }
        
        double explored_ratio = 1.0 - (static_cast<double>(unknown_count) / current_map_->data.size());
        
        if (explored_ratio >= threshold_) {
            RCLCPP_INFO(get_logger(), "Exploration complete! Saving map...");
            saveMap();
        }
    }
    
    void saveMap()
    {
        std::string command = "ros2 run nav2_map_server map_saver_cli -f " + map_save_path_;
        if (system(command.c_str()) == 0) {
            RCLCPP_INFO(get_logger(), "✓ Map saved to %s", map_save_path_.c_str());
            map_saved_ = true;
        }
    }
    
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr check_timer_;
    std::string map_save_path_;
    double threshold_;
    bool map_saved_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapSaverNode>());
    rclcpp::shutdown();
    return 0;
}