#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vector>
#include <queue>
#include <cmath>

enum class ExplorationState
{
  MAPPING,
  SAVE_MAP,
  COMPLETED
};

struct Cell
{
  int x;
  int y;
  bool visited;
  bool has_walls[4];
};

class ExplorationManager : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  ExplorationManager() 
    : Node("exploration_manager_node"), 
      state_(ExplorationState::MAPPING),
      current_cell_x_(0),
      current_cell_y_(0),
      cell_size_(0.5),
      grid_width_(8),
      grid_height_(8),
      map_received_(false)
  {
    this->declare_parameter("cell_size", 0.5);
    this->declare_parameter("grid_width", 8);
    this->declare_parameter("grid_height", 8);

    cell_size_ = this->get_parameter("cell_size").as_double();
    grid_width_ = this->get_parameter("grid_width").as_int();
    grid_height_ = this->get_parameter("grid_height").as_int();

    grid_.resize(grid_height_, std::vector<Cell>(grid_width_));
    for (int i = 0; i < grid_height_; ++i)
    {
      for (int j = 0; j < grid_width_; ++j)
      {
        grid_[i][j].x = j;
        grid_[i][j].y = i;
        grid_[i][j].visited = false;
        for (int k = 0; k < 4; ++k)
        {
          grid_[i][j].has_walls[k] = false;
        }
      }
    }

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&ExplorationManager::mapCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&ExplorationManager::odomCallback, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

    exploration_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&ExplorationManager::exploreNextCell, this));

    RCLCPP_INFO(this->get_logger(), "Exploration Manager Node initialized");
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr /*msg*/)
  {
    map_received_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_ = msg->pose.pose;
    
    current_cell_x_ = static_cast<int>(current_pose_.position.x / cell_size_);
    current_cell_y_ = static_cast<int>(current_pose_.position.y / cell_size_);

    if (current_cell_x_ >= 0 && current_cell_x_ < grid_width_ &&
        current_cell_y_ >= 0 && current_cell_y_ < grid_height_)
    {
      grid_[current_cell_y_][current_cell_x_].visited = true;
    }
  }

  void exploreNextCell()
  {
    if (state_ == ExplorationState::COMPLETED)
    {
      return;
    }

    if (state_ == ExplorationState::MAPPING)
    {
      bool all_visited = true;
      for (const auto& row : grid_)
      {
        for (const auto& cell : row)
        {
          if (!cell.visited)
          {
            all_visited = false;
            break;
          }
        }
        if (!all_visited) break;
      }

      if (all_visited)
      {
        RCLCPP_INFO(this->get_logger(), "Exploration complete! Saving map...");
        state_ = ExplorationState::SAVE_MAP;
        saveMap();
        state_ = ExplorationState::COMPLETED;
        return;
      }

      auto goal = getNextExplorationGoal();
      sendNavigationGoal(goal);
    }
  }

  geometry_msgs::msg::PoseStamped getNextExplorationGoal()
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();

    std::queue<std::pair<int, int>> frontier;
    std::vector<std::vector<bool>> visited_search(grid_height_, std::vector<bool>(grid_width_, false));

    frontier.push({current_cell_y_, current_cell_x_});
    visited_search[current_cell_y_][current_cell_x_] = true;

    int target_x = current_cell_x_;
    int target_y = current_cell_y_;
    bool found = false;

    while (!frontier.empty() && !found)
    {
      auto [y, x] = frontier.front();
      frontier.pop();

      int dx[] = {0, 1, 0, -1};
      int dy[] = {-1, 0, 1, 0};

      for (int i = 0; i < 4; ++i)
      {
        int nx = x + dx[i];
        int ny = y + dy[i];

        if (nx >= 0 && nx < grid_width_ && ny >= 0 && ny < grid_height_ &&
            !visited_search[ny][nx])
        {
          visited_search[ny][nx] = true;

          if (!grid_[ny][nx].visited)
          {
            target_x = nx;
            target_y = ny;
            found = true;
            break;
          }

          frontier.push({ny, nx});
        }
      }
    }

    goal.pose.position.x = (target_x + 0.5) * cell_size_;
    goal.pose.position.y = (target_y + 0.5) * cell_size_;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Next exploration goal: cell [%d, %d] -> [%.2f, %.2f]",
                target_x, target_y, goal.pose.position.x, goal.pose.position.y);

    return goal;
  }

  void sendNavigationGoal(const geometry_msgs::msg::PoseStamped& goal)
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
      std::bind(&ExplorationManager::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&ExplorationManager::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&ExplorationManager::resultCallback, this, std::placeholders::_1);

    nav_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goalResponseCallback(const GoalHandleNav::SharedPtr& goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedbackCallback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> /*feedback*/)
  {
    // Feedback processing can be added here if needed
  }

  void resultCallback(const GoalHandleNav::WrappedResult& result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Navigation goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Navigation goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Navigation goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
  }

  void saveMap()
  {
    RCLCPP_INFO(this->get_logger(), "Calling map_saver service...");
    
    system("ros2 run nav2_map_server map_saver_cli -f ~/ros2_course_ws_master/src/final_project/maps/maze_map");
    
    RCLCPP_INFO(this->get_logger(), "Map saved successfully!");
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr exploration_timer_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  ExplorationState state_;
  std::vector<std::vector<Cell>> grid_;
  int current_cell_x_, current_cell_y_;
  double cell_size_;
  int grid_width_, grid_height_;
  bool map_received_;
  geometry_msgs::msg::Pose current_pose_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExplorationManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}