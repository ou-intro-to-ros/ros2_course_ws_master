/**
 * @file read_display.cpp
 * @brief Basic OpenCV example: Reading and displaying an image
 * 
 * This program demonstrates:
 * - How to load an image from disk using cv::imread()
 * - How to check if an image loaded successfully
 * - How to access image properties (width, height, channels)
 * - How to display an image in a window using cv::imshow()
 * - How to use ament_index_cpp to find package resources
 */

#include <opencv2/opencv.hpp>                                    // Main OpenCV header - includes core, imgproc, highgui
#include <ament_index_cpp/get_package_share_directory.hpp>       // ROS 2 utility to find package installation path
#include <iostream>                                               // For console output

int main()
{
    std::cout << "=== OpenCV Read and Display Demo ===" << std::endl;
    
    // Get the absolute path to where this package's shared resources are installed
    // This is portable - works on any machine where the package is installed
    // Returns something like: /home/user/ros2_ws/install/opencv_examples/share/opencv_examples
    std::string package_path = ament_index_cpp::get_package_share_directory("opencv_examples");
    
    // Construct the full path to our image file
    // The images/ folder gets installed to the share directory (see CMakeLists.txt)
    std::string image_path = package_path + "/images/robot.jpg";
    
    std::cout << "Loading image from: " << image_path << std::endl;
    
    // cv::imread() loads an image from file
    // - Returns a cv::Mat (Matrix) object containing pixel data
    // - By default, loads in BGR (Blue-Green-Red) color format
    // - Returns an empty Mat if the file doesn't exist or can't be read
    cv::Mat img = cv::imread(image_path);
    
    // Always check if the image loaded successfully
    // cv::Mat::empty() returns true if the matrix has no data
    if (img.empty()) {
        std::cerr << "ERROR: Could not read image!" << std::endl;
        std::cerr << "Make sure you have an image at: " << image_path << std::endl;
        return 1;  // Exit with error code
    }
    
    // Display image properties
    std::cout << "Image loaded successfully!" << std::endl;
    
    // img.cols = number of columns (width in pixels)
    std::cout << "Width: " << img.cols << " pixels" << std::endl;
    
    // img.rows = number of rows (height in pixels)
    std::cout << "Height: " << img.rows << " pixels" << std::endl;
    
    // img.channels() = number of color channels
    // 1 = grayscale, 3 = BGR color, 4 = BGRA (with alpha/transparency)
    std::cout << "Channels: " << img.channels() << std::endl;
    
    // cv::imshow() displays an image in a window
    // - First parameter: window name (title bar text)
    // - Second parameter: the cv::Mat image to display
    // Creates a new window if one with that name doesn't exist
    cv::imshow("Original Image", img);
    
    std::cout << "Press any key to close the window..." << std::endl;
    
    // cv::waitKey(0) waits indefinitely for a keyboard press
    // - Parameter 0 means wait forever
    // - A positive number (e.g., 1000) would wait that many milliseconds
    // - Returns the ASCII code of the key pressed
    cv::waitKey(0);
    
    // Clean up: close all OpenCV windows
    // Good practice to prevent windows from staying open
    cv::destroyAllWindows();
    
    return 0;  // Exit successfully
}