/**
 * @file draw_shapes.cpp
 * @brief OpenCV drawing functions example
 * 
 * This program demonstrates:
 * - Drawing basic shapes (rectangles, circles, lines, polygons)
 * - Adding text to images
 * - Understanding BGR color format
 * - Using different line thicknesses and fill options
 * 
 * Drawing is useful for:
 * - Annotating detection results (bounding boxes, labels)
 * - Creating visualizations and debugging displays
 * - Marking regions of interest
 * - Building simple UIs on images
 */

#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

int main()
{
    std::cout << "=== OpenCV Drawing Shapes Demo ===" << std::endl;
    
    std::string package_path = ament_index_cpp::get_package_share_directory("opencv_examples");
    std::string image_path = package_path + "/images/robot.jpg";
    
    cv::Mat img = cv::imread(image_path);
    
    if (img.empty()) {
        std::cerr << "ERROR: Could not read image!" << std::endl;
        return 1;
    }
    
    // Create a copy to draw on (preserves original)
    // .clone() creates a deep copy of the image data
    cv::Mat canvas = img.clone();
    
    // === DRAW RECTANGLE ===
    // cv::rectangle(image, top_left_corner, bottom_right_corner, color, thickness)
    // - cv::Point(x, y) defines corner positions
    // - cv::Scalar(B, G, R) defines color in BGR format
    //   - B=0, G=255, R=0 = pure green
    // - thickness: line width in pixels (positive = outline, -1 = filled)
    cv::rectangle(canvas, 
                  cv::Point(50, 50),           // Top-left corner
                  cv::Point(200, 200),         // Bottom-right corner
                  cv::Scalar(0, 255, 0),       // Green color (BGR)
                  3);                          // 3-pixel thick outline
    std::cout << "Drew rectangle" << std::endl;
    
    // === DRAW FILLED CIRCLE ===
    // cv::circle(image, center, radius, color, thickness)
    // Using thickness=-1 fills the circle
    cv::circle(canvas, 
               cv::Point(canvas.cols - 100, 100),  // Center: near top-right
               50,                                  // Radius: 50 pixels
               cv::Scalar(255, 0, 0),              // Blue color (BGR)
               -1);                                 // -1 means filled
    std::cout << "Drew filled circle" << std::endl;
    
    // === DRAW LINE ===
    // cv::line(image, start_point, end_point, color, thickness)
    cv::line(canvas, 
             cv::Point(50, canvas.rows - 50),           // Start: bottom-left
             cv::Point(canvas.cols - 50, canvas.rows - 50),  // End: bottom-right
             cv::Scalar(0, 0, 255),                     // Red color (BGR)
             5);                                        // 5-pixel thick line
    std::cout << "Drew line" << std::endl;
    
    // === DRAW TEXT ===
    // cv::putText(image, text, position, font, scale, color, thickness)
    // - position: bottom-left corner of the text string
    // - font: one of several predefined fonts (HERSHEY_SIMPLEX is most common)
    // - scale: font size multiplier (1.0 = base size)
    cv::putText(canvas, 
                "OpenCV Demo",                    // Text to display
                cv::Point(50, canvas.rows - 80),  // Position: bottom-left of text
                cv::FONT_HERSHEY_SIMPLEX,         // Font style
                1.0,                              // Font scale
                cv::Scalar(255, 255, 255),        // White color (BGR)
                2);                               // 2-pixel thick text
    std::cout << "Drew text" << std::endl;
    
    // === DRAW POLYGON ===
    // cv::polylines() can draw arbitrary polygons
    // - Define points as a vector of cv::Point
    // - isClosed=true connects last point back to first
    std::vector<cv::Point> triangle_points = {
        cv::Point(canvas.cols/2, 50),         // Top vertex
        cv::Point(canvas.cols/2 - 50, 150),   // Bottom-left vertex
        cv::Point(canvas.cols/2 + 50, 150)    // Bottom-right vertex
    };
    cv::polylines(canvas,           // Image to draw on
                  triangle_points,  // Vector of points
                  true,             // true = closed polygon (connect last to first)
                  cv::Scalar(255, 255, 0),  // Cyan color (BGR)
                  3);               // 3-pixel thick lines
    std::cout << "Drew triangle" << std::endl;
    
    // === DISPLAY COMPARISON ===
    cv::imshow("Original", img);      // Original image without drawings
    cv::imshow("With Shapes", canvas); // Image with all drawings
    
    std::cout << "Press any key to close all windows..." << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}