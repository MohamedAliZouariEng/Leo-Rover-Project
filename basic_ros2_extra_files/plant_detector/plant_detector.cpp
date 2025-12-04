#include "leo_rover_tasks/plant_detector.hpp"
#include <iostream>
#include <random>

/**
 * ALTERNATIVE SIMPLE IMPLEMENTATION
 *
 * This is a lightweight alternative that doesn't require LibTorch.
 * It uses simple computer vision techniques to detect "plant-like" features.
 * This is perfect for educational purposes where you want to focus on ROS2
 * concepts rather than deep learning complexity.
 */

PlantDetector::PlantDetector(const std::string &model_path)
    : device_(CPU), model_loaded_(true) // Set to true for simple implementation
{
  std::cout << "PlantDetector: Using simple computer vision implementation"
            << std::endl;
  std::cout << "PlantDetector: Model path (ignored in simple mode): "
            << model_path << std::endl;
}

float PlantDetector::predict(const cv::Mat &image) {
  if (image.empty()) {
    std::cerr << "PlantDetector: Empty image provided, returning 0.0"
              << std::endl;
    return 0.0f;
  }

  try {
    // Simple plant detection using color-based analysis
    float confidence = detect_green_objects(image);

    // Add some randomness to make it more realistic for demo purposes
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> noise(-0.1f, 0.1f);

    confidence += noise(gen);
    confidence = std::max(0.0f, std::min(1.0f, confidence));

    return confidence;
  } catch (const std::exception &e) {
    std::cerr << "PlantDetector: Prediction error: " << e.what() << std::endl;
    return 0.0f;
  }
}

float PlantDetector::detect_green_objects(const cv::Mat &image) {
  // Convert to HSV for better color detection
  cv::Mat hsv_image;
  cv::cvtColor(image, hsv_image, cv::COLOR_RGB2HSV);

  // Define green color range in HSV
  cv::Scalar lower_green(35, 50, 50);   // Lower bound for green
  cv::Scalar upper_green(80, 255, 255); // Upper bound for green

  // Create mask for green colors
  cv::Mat green_mask;
  cv::inRange(hsv_image, lower_green, upper_green, green_mask);

  // Apply morphological operations to reduce noise
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::morphologyEx(green_mask, green_mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(green_mask, green_mask, cv::MORPH_CLOSE, kernel);

  // Calculate the percentage of green pixels
  int total_pixels = image.rows * image.cols;
  int green_pixels = cv::countNonZero(green_mask);
  float green_ratio = static_cast<float>(green_pixels) / total_pixels;

  // Find contours to detect plant-like shapes
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(green_mask, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  float shape_score = 0.0f;
  if (!contours.empty()) {
    // Analyze the largest green contour
    auto largest_contour = *std::max_element(
        contours.begin(), contours.end(),
        [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
          return cv::contourArea(a) < cv::contourArea(b);
        });

    double area = cv::contourArea(largest_contour);
    double perimeter = cv::arcLength(largest_contour, true);

    if (perimeter > 0) {
      // Calculate circularity (4π * area / perimeter²)
      double circularity = 4 * M_PI * area / (perimeter * perimeter);

      // Plants tend to have irregular shapes (low circularity)
      // and reasonable size
      if (area > 500 && circularity < 0.7) { // Irregular shape
        shape_score = 0.4f;
      }
    }
  }

  // Combine color and shape analysis
  float confidence = std::min(1.0f, green_ratio * 3.0f + shape_score);

  // Add texture analysis for more sophisticated detection
  confidence += analyze_texture(image) * 0.2f;

  return std::min(1.0f, confidence);
}

float PlantDetector::analyze_texture(const cv::Mat &image) {
  // Convert to grayscale for texture analysis
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_RGB2GRAY);

  // Calculate standard deviation as a simple texture measure
  cv::Scalar mean, stddev;
  cv::meanStdDev(gray_image, mean, stddev);

  // Plants typically have moderate texture variation
  float texture_score = stddev[0] / 50.0f; // Normalize roughly
  return std::min(1.0f, texture_score);
}