/**
 * @file blur_smooth.cpp
 * @brief OpenCV image smoothing and blurring example
 * 
 * This program demonstrates:
 * - Different types of blur/smoothing filters
 * - When to use each type of blur
 * - How filter kernel size affects the result
 * 
 * Blurring is used for:
 * - Noise reduction
 * - Preprocessing before edge detection
 * - Creating artistic effects
 * - Downsampling preparation
 * - Background/foreground separation
 * 
 * Filter Types:
 * 1. Averaging: Simple mean of pixels (fast, uniform blur)
 * 2. Gaussian: Weighted average (natural blur, emphasizes center)
 * 3. Median: Takes median value (excellent for salt-and-pepper noise)
 * 4. Bilateral: Preserves edges while smoothing (best for noise reduction with edge preservation)
 */

#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

int main()
{
    std::cout << "=== OpenCV Blur and Smoothing Demo ===" << std::endl;
    
    std::string package_path = ament_index_cpp::get_package_share_directory("opencv_examples");
    std::string image_path = package_path + "/images/robot.jpg";
    
    cv::Mat img = cv::imread(image_path);
    
    if (img.empty()) {
        std::cerr << "ERROR: Could not read image!" << std::endl;
        return 1;
    }
    
    // === AVERAGING BLUR ===
    // Simple box filter - replaces each pixel with the average of its neighbors
    // cv::blur(source, destination, kernel_size)
    // - kernel_size: cv::Size(width, height) of the averaging window
    // - Larger kernel = more blur
    // - Fast but produces uniform blur
    cv::Mat blur_avg;
    cv::blur(img, blur_avg, cv::Size(9, 9));  // 9x9 pixel averaging window
    std::cout << "Applied averaging blur (9x9)" << std::endl;
    // Use case: Quick smoothing when edge preservation isn't critical
    
    // === GAUSSIAN BLUR ===
    // Uses a Gaussian (bell curve) kernel - center pixels weighted more heavily
    // cv::GaussianBlur(source, destination, kernel_size, sigma)
    // - kernel_size: must be odd numbers (e.g., 3, 5, 7, 9)
    // - sigma: standard deviation (0 = auto-calculate from kernel size)
    // - Produces more natural-looking blur than averaging
    cv::Mat blur_gauss;
    cv::GaussianBlur(img, blur_gauss, cv::Size(9, 9), 0);
    std::cout << "Applied Gaussian blur (9x9)" << std::endl;
    // Use case: Most common blur, good for general smoothing and preprocessing
    
    // === MEDIAN BLUR ===
    // Replaces each pixel with the median value of its neighbors
    // cv::medianBlur(source, destination, kernel_size)
    // - kernel_size: single number (creates square kernel), must be odd
    // - Excellent at removing "salt and pepper" noise (random white/black pixels)
    // - Preserves edges better than averaging
    cv::Mat blur_median;
    cv::medianBlur(img, blur_median, 9);  // 9x9 pixel window
    std::cout << "Applied median blur (9x9)" << std::endl;
    // Use case: Removing impulse noise while keeping edges sharp
    
    // === BILATERAL FILTER ===
    // Advanced filter that smooths while preserving edges
    // cv::bilateralFilter(source, destination, diameter, sigma_color, sigma_space)
    // - diameter: pixel neighborhood diameter (or -1 for auto)
    // - sigma_color: filter sigma in color space (larger = more colors mixed)
    // - sigma_space: filter sigma in coordinate space (larger = farther pixels influence)
    // 
    // How it works:
    // - Only averages pixels that are similar in color (preserves edges)
    // - Slower than other methods but produces best results
    cv::Mat blur_bilateral;
    cv::bilateralFilter(img, blur_bilateral, 9, 75, 75);
    std::cout << "Applied bilateral filter" << std::endl;
    // Use case: Noise reduction while keeping edges sharp (e.g., portrait smoothing)
    
    // === DISPLAY ALL RESULTS ===
    // Notice how each blur type affects the image differently
    cv::imshow("Original", img);
    cv::imshow("Averaging Blur", blur_avg);        // Uniform, simple blur
    cv::imshow("Gaussian Blur", blur_gauss);       // Natural-looking blur
    cv::imshow("Median Blur", blur_median);        // Good for noise, preserves edges
    cv::imshow("Bilateral Filter", blur_bilateral); // Smooths but keeps edges sharp
    
    std::cout << "Press any key to close all windows..." << std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}