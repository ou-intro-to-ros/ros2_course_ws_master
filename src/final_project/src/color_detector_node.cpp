#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ColorDetector : public rclcpp::Node
{
public:
  ColorDetector() : Node("color_detector_node")
  {
    this->declare_parameter("camera_topic", "/camera/image_raw");
    std::string camera_topic = this->get_parameter("camera_topic").as_string();

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_topic, 10,
      std::bind(&ColorDetector::imageCallback, this, std::placeholders::_1));

    color_pub_ = this->create_publisher<std_msgs::msg::String>("/floor_color", 10);

    RCLCPP_INFO(this->get_logger(), "Color Detector Node initialized");
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

    int height = cv_ptr->image.rows;
    int width = cv_ptr->image.cols;
    cv::Rect floor_roi(0, height * 2 / 3, width, height / 3);
    cv::Mat floor_region = cv_ptr->image(floor_roi);

    std::string color = detectDominantColor(floor_region);

    std_msgs::msg::String color_msg;
    color_msg.data = color;
    color_pub_->publish(color_msg);
  }

  std::string detectDominantColor(const cv::Mat& image)
  {
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    cv::Scalar mean_color = cv::mean(hsv);
    double hue = mean_color[0];
    double saturation = mean_color[1];
    double value = mean_color[2];

    if (saturation < 50)
    {
      if (value > 200) return "white";
      if (value < 50) return "black";
      return "gray";
    }

    if (hue < 15 || hue > 165) return "red";
    if (hue >= 15 && hue < 35) return "orange";
    if (hue >= 35 && hue < 75) return "yellow";
    if (hue >= 75 && hue < 150) return "green";
    if (hue >= 150 && hue < 165) return "cyan";
    if (hue >= 95 && hue < 135) return "blue";
    if (hue >= 135 && hue < 165) return "purple";

    return "unknown";
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ColorDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}