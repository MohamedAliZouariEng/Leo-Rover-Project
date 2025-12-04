#ifndef PLANT_DETECTOR_HPP
#define PLANT_DETECTOR_HPP

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

// Uncomment these lines if you want to use the LibTorch version instead
// #include <torch/torch.h>
// #include <torch/script.h>

class PlantDetector {
public:
  /**
   * @brief Constructor for PlantDetector
   * @param model_path Path to the model file (for LibTorch version) or ignored
   * (for simple version)
   */
  explicit PlantDetector(const std::string &model_path);

  /**
   * @brief Destructor
   */
  ~PlantDetector() = default;

  /**
   * @brief Predict if an image contains a plant
   * @param image OpenCV Mat image in RGB format
   * @return Float confidence score (0.0 to 1.0)
   */
  float predict(const cv::Mat &image);

private:
  // For LibTorch version, uncomment these:
  /*
  torch::Tensor preprocess_image(const cv::Mat& image);
  float postprocess_output(const torch::Tensor& output);
  torch::jit::script::Module model_;
  torch::Device device_;
  bool model_loaded_;
  static constexpr int INPUT_HEIGHT = 150;
  static constexpr int INPUT_WIDTH = 150;
  const std::vector<float> mean_{0.485f, 0.456f, 0.406f};
  const std::vector<float> std_{0.229f, 0.224f, 0.225f};
  */

  // For simple computer vision version:
  /**
   * @brief Simple computer vision based plant detection
   * @param image Input OpenCV Mat image
   * @return Confidence score based on color and shape analysis
   */
  float detect_green_objects(const cv::Mat &image);

  /**
   * @brief Analyze texture features of the image
   * @param image Input OpenCV Mat image
   * @return Texture score contributing to plant detection
   */
  float analyze_texture(const cv::Mat &image);

  // Simple version state variables
  bool model_loaded_;

  // Dummy device type for interface compatibility
  enum DeviceType { CPU, CUDA };
  DeviceType device_;
};

#endif // PLANT_DETECTOR_HPP