#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

enum class ShapeType
{
  UNKNOWN,
  SQUARE,
  CIRCLE
};

struct DetectedShape
{
  ShapeType type;
  cv::Point2f center;
  double confidence;
  double distance;
};

class ShapeDetector : public rclcpp::Node
{
public:
  ShapeDetector() : Node("shape_detector_node")
  {
    this->declare_parameter("camera_topic", "/camera/image_raw");
    this->declare_parameter("confidence_threshold", 0.7);
    this->declare_parameter("min_contour_area", 500.0);

    std::string camera_topic = this->get_parameter("camera_topic").as_string();
    confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
    min_contour_area_ = this->get_parameter("min_contour_area").as_double();

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_topic, 10,
      std::bind(&ShapeDetector::imageCallback, this, std::placeholders::_1));

    shape_pub_ = this->create_publisher<std_msgs::msg::String>("/detected_shape", 10);
    shape_location_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/shape_location", 10);

    RCLCPP_INFO(this->get_logger(), "Shape Detector Node initialized");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
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

    cv::Mat gray, blurred, edges;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
    cv::Canny(blurred, edges, 50, 150);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours)
    {
      double area = cv::contourArea(contour);
      if (area < min_contour_area_)
      {
        continue;
      }

      double confidence = 0.0;
      ShapeType type = classifyContour(contour, confidence);

      if (confidence >= confidence_threshold_ && type != ShapeType::UNKNOWN)
      {
        cv::Moments M = cv::moments(contour);
        if (M.m00 != 0)
        {
          cv::Point2f center(M.m10 / M.m00, M.m01 / M.m00);

          cv::Rect bbox = cv::boundingRect(contour);
          double distance = estimateDistance(bbox.height);

          std_msgs::msg::String shape_msg;
          if (type == ShapeType::SQUARE)
          {
            shape_msg.data = "square";
            cv::rectangle(cv_ptr->image, bbox, cv::Scalar(0, 255, 0), 2);
          }
          else if (type == ShapeType::CIRCLE)
          {
            shape_msg.data = "circle";
            cv::circle(cv_ptr->image, center, bbox.width / 2, cv::Scalar(255, 0, 0), 2);
          }

          shape_pub_->publish(shape_msg);

          geometry_msgs::msg::PointStamped point_msg;
          point_msg.header.stamp = this->now();
          point_msg.header.frame_id = "camera_link";
          point_msg.point.x = distance;
          point_msg.point.y = (center.x - cv_ptr->image.cols / 2.0) * 0.001;
          point_msg.point.z = 0.0;
          shape_location_pub_->publish(point_msg);

          RCLCPP_INFO(this->get_logger(), "Detected %s with confidence %.2f at distance %.2fm",
                      shape_msg.data.c_str(), confidence, distance);
        }
      }
    }
  }

  ShapeType classifyContour(const std::vector<cv::Point>& contour, double& confidence)
  {
    std::vector<cv::Point> approx;
    double epsilon = 0.04 * cv::arcLength(contour, true);
    cv::approxPolyDP(contour, approx, epsilon, true);

    int vertices = approx.size();
    double area = cv::contourArea(contour);
    double perimeter = cv::arcLength(contour, true);
    double circularity = 4 * M_PI * area / (perimeter * perimeter);

    if (vertices == 4)
    {
      cv::Rect bbox = cv::boundingRect(contour);
      double aspect_ratio = static_cast<double>(bbox.width) / bbox.height;
      
      if (aspect_ratio >= 0.85 && aspect_ratio <= 1.15)
      {
        confidence = 1.0 - std::abs(1.0 - aspect_ratio);
        return ShapeType::SQUARE;
      }
    }
    else if (circularity > 0.7)
    {
      confidence = circularity;
      return ShapeType::CIRCLE;
    }

    confidence = 0.0;
    return ShapeType::UNKNOWN;
  }

  double estimateDistance(double pixel_height)
  {
    const double KNOWN_HEIGHT = 0.2;
    const double FOCAL_LENGTH = 500.0;
    
    if (pixel_height > 0)
    {
      return (KNOWN_HEIGHT * FOCAL_LENGTH) / pixel_height;
    }
    return 0.0;
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shape_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr shape_location_pub_;

  double confidence_threshold_;
  double min_contour_area_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ShapeDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}