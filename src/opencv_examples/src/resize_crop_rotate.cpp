/**
 * @file resize_crop_rotate.cpp
 * @brief OpenCV geometric transformations example
 * 
 * This program demonstrates:
 * - Resizing images (making them larger or smaller)
 * - Cropping (extracting a rectangular region)
 * - Rotating images around a center point
 * - Flipping images (mirroring)
 * 
 * These operations are fundamental for:
 * - Preprocessing images for machine learning (standardizing sizes)
 * - Data augmentation (creating variations of training data)
 * - Image alignment and registration
 * - Creating thumbnails or previews
 */

#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

int main()
{
    std::cout << "=== OpenCV Resize, Crop, and Rotate Demo ===" << std::endl;
    
    std::string package_path = ament_index_cpp::get_package_share_directory("opencv_examples");
    std::string image_path = package_path + "/images/robot.jpg";
    
    cv::Mat img = cv::imread(image_path);
    
    if (img.empty()) {
        std::cerr << "ERROR: Could not read image!" << std::endl;
        return 1;
    }
    
    // === RESIZE - ABSOLUTE SIZE ===
    // cv::resize() changes the image dimensions
    // Syntax: resize(source, destination, new_size)
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(320, 240));  // Resize to exactly 320x240 pixels
    std::cout << "Resized to 320x240" << std::endl;
    // Note: This may distort the image if the aspect ratio changes
    
    // === RESIZE - SCALING FACTOR ===
    // You can also resize by a scale factor instead of absolute size
    // Syntax: resize(source, destination, Size(), fx, fy)
    // - cv::Size() empty means "calculate from scale factors"
    // - fx = horizontal scale factor
    // - fy = vertical scale factor
    cv::Mat enlarged;
    cv::resize(img, enlarged, cv::Size(), 2.0, 2.0);  // Double both dimensions
    std::cout << "Enlarged by 2x" << std::endl;
    // Using the same factor for both preserves aspect ratio
    
    // === CROP - REGION OF INTEREST (ROI) ===
    // Cropping extracts a rectangular region from the image
    // cv::Rect(x, y, width, height) defines the rectangle
    // - (x, y) is the top-left corner
    // - width and height define the size
    int crop_width = img.cols / 2;   // Half the original width
    int crop_height = img.rows / 2;  // Half the original height
    int start_x = img.cols / 4;      // Start 1/4 from the left
    int start_y = img.rows / 4;      // Start 1/4 from the top
    
    cv::Rect roi(start_x, start_y, crop_width, crop_height);
    cv::Mat cropped = img(roi);  // Use () operator to extract the ROI
    std::cout << "Cropped center region" << std::endl;
    // Note: This creates a reference, not a copy. Use .clone() if you need a copy.
    
    // === ROTATE ===
    // Rotation requires creating a transformation matrix first
    // Step 1: Define the center point for rotation
    cv::Point2f center(img.cols / 2.0, img.rows / 2.0);  // Center of the image
    
    // Step 2: Create a rotation matrix
    // getRotationMatrix2D(center, angle, scale)
    // - center: Point to rotate around
    // - angle: Rotation angle in degrees (positive = counter-clockwise)
    // - scale: Scaling factor (1.0 = no scaling)
    cv::Mat rot_mat = cv::getRotationMatrix2D(center, 45, 1.0);
    
    // Step 3: Apply the transformation
    // warpAffine applies a 2D affine transformation
    cv::Mat rotated;
    cv::warpAffine(img, rotated, rot_mat, img.size());
    std::cout << "Rotated 45 degrees" << std::endl;
    // Note: Corners may be cut off; use a larger output size if needed
    
    // === FLIP ===
    // cv::flip() mirrors the image
    // Syntax: flip(source, destination, flipCode)
    // flipCode:
    //   0 = flip vertically (upside down)
    //   1 = flip horizontally (mirror)
    //   -1 = flip both (180 degree rotation)
    cv::Mat flipped;
    cv::flip(img, flipped, 1);  // Horizontal flip
    std::cout << "Flipped horizontally" << std::endl;
    
    // === DISPLAY ALL TRANSFORMATIONS ===
    cv::imshow("Original", img);
    cv::imshow("Resized (320x240)", resized);
    cv::imshow("Cropped", cropped);
    cv::imshow("Rotated 45°", rotated);
    cv::imshow("Flipped", flipped);
    
    std::cout << "Press any key to close all windows..." << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}