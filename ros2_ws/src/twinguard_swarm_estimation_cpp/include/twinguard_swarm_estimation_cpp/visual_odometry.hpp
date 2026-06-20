#pragma once

#include <array>
#include <optional>
#include <vector>

#include <opencv2/opencv.hpp>

namespace twinguard::estimation
{

struct VisualOdometryEstimate
{
  std::array<double, 3> velocity_estimate{0.0, 0.0, 0.0};
  double quality{0.0};
  int tracked_features{0};
  double mean_tracking_error{0.0};
};

class SparseOpticalFlowVO
{
public:
  explicit SparseOpticalFlowVO(int max_features = 200);

  std::optional<VisualOdometryEstimate> process_frame(
    const cv::Mat & gray_frame,
    const cv::Mat & depth_frame,
    double dt_s);

private:
  void reseed_features(const cv::Mat & gray_frame);
  double estimate_depth_scale(const cv::Mat & depth_frame) const;

  cv::Mat previous_frame_;
  std::vector<cv::Point2f> previous_points_;
  int max_features_;
  bool has_previous_{false};
  double fallback_m_per_pixel_{0.01};
  double focal_length_px_{320.0};
};

}  // namespace twinguard::estimation
