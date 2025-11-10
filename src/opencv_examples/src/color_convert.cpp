/**
 * @file color_convert.cpp
 * @brief OpenCV color space conversion example
 * 
 * This program demonstrates:
 * - Converting between different color spaces (BGR, Grayscale, HSV)
 * - Why different color spaces are useful in computer vision
 * - How to split multi-channel images into separate channels
 * - Displaying multiple images simultaneously
 * 
 * Color Spaces Explained:
 * - BGR: Blue-Green-Red (OpenCV's default, opposite of RGB)
 * - Grayscale: Single channel, intensity only (0=black, 255=white)
 * - HSV: Hue-Saturation-Value (better for color-based detection)
 *   - Hue: The color type (0-179 in OpenCV, represents 0-360 degrees)
 *   - Saturation: Color intensity (0=gray, 255=full color)
 *   - Value: Brightness (0=black, 255=bright)
 */

#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

int main()
{
    std::cout << "=== OpenCV Color Conversion Demo ===" << std::endl;
    
    // Get package path and construct image path
    std::string package_path = ament_index_cpp::get_package_share_directory("opencv_examples");
    std::string image_path = package_path + "/images/robot.jpg";
    
    // Load the image in default BGR format
    cv::Mat img = cv::imread(image_path);
    
    if (img.empty()) {
        std::cerr << "ERROR: Could not read image!" << std::endl;
        return 1;
    }
    
    // === GRAYSCALE CONVERSION ===
    // Grayscale is useful for:
    // - Reducing data (1 channel instead of 3)
    // - Edge detection algorithms
    // - Template matching
    // - When color information isn't needed
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    std::cout << "Converted to grayscale" << std::endl;
    
    // === HSV CONVERSION ===
    // HSV is useful for:
    // - Color-based object detection (e.g., finding all red objects)
    // - Color thresholding (more intuitive than BGR)
    // - Lighting-invariant processing (V channel separates brightness)
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    std::cout << "Converted to HSV" << std::endl;
    
    // === CHANNEL SPLITTING ===
    // Split the 3-channel HSV image into three separate 1-channel images
    // This allows you to work with H, S, and V independently
    std::vector<cv::Mat> hsv_channels;  // Vector to hold the three channels
    cv::split(hsv, hsv_channels);       // Split into separate channels
    // hsv_channels[0] = Hue channel
    // hsv_channels[1] = Saturation channel
    // hsv_channels[2] = Value channel
    std::cout << "Split HSV into channels (H, S, V)" << std::endl;
    
    // === DISPLAY ALL IMAGES ===
    // You can have multiple windows open at once
    cv::imshow("Original (BGR)", img);
    cv::imshow("Grayscale", gray);
    cv::imshow("HSV", hsv);  // Note: Will look weird because it's still interpreted as BGR when displayed
    
    // Display individual HSV channels
    // These appear as grayscale images showing the intensity of each component
    cv::imshow("Hue Channel", hsv_channels[0]);         // Shows color information
    cv::imshow("Saturation Channel", hsv_channels[1]); // Shows color purity
    cv::imshow("Value Channel", hsv_channels[2]);      // Shows brightness
    
    std::cout << "Press any key to close all windows..." << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}