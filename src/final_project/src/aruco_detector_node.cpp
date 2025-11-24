#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <map>

struct ArucoMarkerInfo
{
  int id;
  geometry_msgs::msg::Pose pose;
  rclcpp::Time last_seen;
  bool verified;
};

class ArucoDetector : public rclcpp::Node
{
public:
  ArucoDetector() : Node("aruco_detector_node"), camera_info_received_(false)
  {
    this->declare_parameter("camera_topic", "/camera/image_raw");
    this->declare_parameter("camera_info_topic", "/camera/camera_info");
    this->declare_parameter("aruco_dictionary", "DICT_4X4_50");
    this->declare_parameter("marker_size", 0.1);

    std::string camera_topic = this->get_parameter("camera_topic").as_string();
    std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
    marker_size_ = this->get_parameter("marker_size").as_double();

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_topic, 10,
      std::bind(&ArucoDetector::imageCallback, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, 10,
      std::bind(&ArucoDetector::cameraInfoCallback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/aruco_markers", 10);

    aruco_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/detected_aruco_pose", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    detector_params_ = cv::aruco::DetectorParameters::create();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ArucoDetector::publishMarkers, this));

    RCLCPP_INFO(this->get_logger(), "ArUco Detector Node initialized");
  }

private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (!camera_info_received_)
    {
      camera_matrix_ = cv::Mat(3, 3, CV_64F);
      for (int i = 0; i < 9; i++)
      {
        camera_matrix_.at<double>(i / 3, i % 3) = msg->k[i];
      }

      dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
      for (size_t i = 0; i < msg->d.size(); i++)
      {
        dist_coeffs_.at<double>(i) = msg->d[i];
      }

      camera_info_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Camera calibration info received");
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!camera_info_received_)
    {
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(cv_ptr->image, dictionary_, marker_corners, marker_ids, detector_params_);

    if (marker_ids.size() > 0)
    {
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

      for (size_t i = 0; i < marker_ids.size(); i++)
      {
        geometry_msgs::msg::Pose pose;
        pose.position.x = tvecs[i][0];
        pose.position.y = tvecs[i][1];
        pose.position.z = tvecs[i][2];

        cv::Mat rotation_matrix;
        cv::Rodrigues(rvecs[i], rotation_matrix);
        
        double sy = sqrt(rotation_matrix.at<double>(0,0) * rotation_matrix.at<double>(0,0) +
                         rotation_matrix.at<double>(1,0) * rotation_matrix.at<double>(1,0));
        
        tf2::Quaternion q;
        if (sy > 1e-6)
        {
          double roll = atan2(rotation_matrix.at<double>(2,1), rotation_matrix.at<double>(2,2));
          double pitch = atan2(-rotation_matrix.at<double>(2,0), sy);
          double yaw = atan2(rotation_matrix.at<double>(1,0), rotation_matrix.at<double>(0,0));
          q.setRPY(roll, pitch, yaw);
        }
        else
        {
          double roll = atan2(-rotation_matrix.at<double>(1,2), rotation_matrix.at<double>(1,1));
          double pitch = atan2(-rotation_matrix.at<double>(2,0), sy);
          double yaw = 0;
          q.setRPY(roll, pitch, yaw);
        }

        pose.orientation = tf2::toMsg(q);

        updateMarkerDatabase(marker_ids[i], pose);

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->now();
        pose_stamped.header.frame_id = "camera_link";
        pose_stamped.pose = pose;
        aruco_pose_pub_->publish(pose_stamped);

        RCLCPP_INFO(this->get_logger(), "Detected ArUco ID: %d at position [%.2f, %.2f, %.2f]",
                    marker_ids[i], tvecs[i][0], tvecs[i][1], tvecs[i][2]);
      }

      cv::aruco::drawDetectedMarkers(cv_ptr->image, marker_corners, marker_ids);
    }
  }

  void updateMarkerDatabase(int id, const geometry_msgs::msg::Pose& pose)
  {
    auto it = detected_markers_.find(id);
    if (it != detected_markers_.end())
    {
      it->second.pose = pose;
      it->second.last_seen = this->now();
    }
    else
    {
      ArucoMarkerInfo info;
      info.id = id;
      info.pose = pose;
      info.last_seen = this->now();
      info.verified = false;
      detected_markers_[id] = info;
    }
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    int idx = 0;

    for (const auto& pair : detected_markers_)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "camera_link";
      marker.header.stamp = this->now();
      marker.ns = "aruco_markers";
      marker.id = idx++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = pair.second.pose;
      marker.scale.x = marker_size_;
      marker.scale.y = marker_size_;
      marker.scale.z = 0.01;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      marker_array.markers.push_back(marker);
    }

    marker_pub_->publish(marker_array);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  bool camera_info_received_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  double marker_size_;
  std::map<int, ArucoMarkerInfo> detected_markers_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}