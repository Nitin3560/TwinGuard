#include "twinguard_swarm_estimation_cpp/visual_odometry.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace twinguard::estimation
{

SparseOpticalFlowVO::SparseOpticalFlowVO(int max_features)
: max_features_(std::max(max_features, 20))
{
}

std::optional<VisualOdometryEstimate> SparseOpticalFlowVO::process_frame(
  const cv::Mat & gray_frame,
  const cv::Mat & depth_frame,
  double dt_s)
{
  if (gray_frame.empty()) {
    return std::nullopt;
  }

  cv::Mat gray;
  if (gray_frame.channels() == 1) {
    gray = gray_frame;
  } else {
    cv::cvtColor(gray_frame, gray, cv::COLOR_BGR2GRAY);
  }

  if (!has_previous_ || previous_points_.empty()) {
    previous_frame_ = gray.clone();
    reseed_features(previous_frame_);
    has_previous_ = true;
    return std::nullopt;
  }

  std::vector<cv::Point2f> next_points;
  std::vector<unsigned char> status;
  std::vector<float> errors;
  cv::calcOpticalFlowPyrLK(previous_frame_, gray, previous_points_, next_points, status, errors);

  double sum_dx = 0.0;
  double sum_dy = 0.0;
  double sum_error = 0.0;
  int tracked = 0;
  for (std::size_t i = 0; i < status.size(); ++i) {
    if (!status[i]) {
      continue;
    }
    sum_dx += static_cast<double>(next_points[i].x - previous_points_[i].x);
    sum_dy += static_cast<double>(next_points[i].y - previous_points_[i].y);
    sum_error += i < errors.size() ? static_cast<double>(errors[i]) : 0.0;
    ++tracked;
  }

  previous_frame_ = gray.clone();
  previous_points_.clear();
  for (std::size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      previous_points_.push_back(next_points[i]);
    }
  }
  if (static_cast<int>(previous_points_.size()) < max_features_ * 3 / 10) {
    reseed_features(previous_frame_);
  }

  if (tracked < 8 || dt_s <= 1e-4) {
    return std::nullopt;
  }

  const double mean_dx = sum_dx / static_cast<double>(tracked);
  const double mean_dy = sum_dy / static_cast<double>(tracked);
  const double mean_error = sum_error / static_cast<double>(tracked);
  const double track_fraction = std::clamp(
    static_cast<double>(tracked) / static_cast<double>(max_features_), 0.0, 1.0);
  const double error_quality = 1.0 / (1.0 + mean_error / 10.0);
  const double quality = std::clamp(track_fraction * error_quality, 0.0, 1.0);
  const double m_per_pixel = estimate_depth_scale(depth_frame);

  VisualOdometryEstimate estimate;
  estimate.velocity_estimate = {
    mean_dx * m_per_pixel / dt_s,
    mean_dy * m_per_pixel / dt_s,
    0.0,
  };
  estimate.quality = quality;
  estimate.tracked_features = tracked;
  estimate.mean_tracking_error = mean_error;
  return estimate;
}

void SparseOpticalFlowVO::reseed_features(const cv::Mat & gray_frame)
{
  previous_points_.clear();
  cv::goodFeaturesToTrack(
    gray_frame,
    previous_points_,
    max_features_,
    0.01,
    8.0);
}

double SparseOpticalFlowVO::estimate_depth_scale(const cv::Mat & depth_frame) const
{
  if (depth_frame.empty()) {
    return fallback_m_per_pixel_;
  }

  cv::Scalar mean_scalar;
  if (depth_frame.type() == CV_32FC1) {
    cv::Mat finite_mask = depth_frame == depth_frame;
    mean_scalar = cv::mean(depth_frame, finite_mask);
  } else if (depth_frame.type() == CV_16UC1) {
    mean_scalar = cv::mean(depth_frame);
    mean_scalar[0] *= 0.001;
  } else {
    return fallback_m_per_pixel_;
  }

  const double depth_m = mean_scalar[0];
  if (!std::isfinite(depth_m) || depth_m <= 0.05) {
    return fallback_m_per_pixel_;
  }
  return std::clamp(depth_m / focal_length_px_, 1e-4, 0.1);
}

}  // namespace twinguard::estimation
