#include "twinguard_swarm_estimation_cpp/visual_odometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
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

  double sum_vx_m = 0.0;
  double sum_vy_m = 0.0;
  double sum_error = 0.0;
  int tracked = 0;
  for (std::size_t i = 0; i < status.size(); ++i) {
    if (!status[i]) {
      continue;
    }
    const double m_per_pixel = estimate_depth_scale_at(depth_frame, previous_points_[i]);
    sum_vx_m += static_cast<double>(next_points[i].x - previous_points_[i].x) * m_per_pixel;
    sum_vy_m += static_cast<double>(next_points[i].y - previous_points_[i].y) * m_per_pixel;
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

  const double mean_vx_m = sum_vx_m / static_cast<double>(tracked);
  const double mean_vy_m = sum_vy_m / static_cast<double>(tracked);
  const double mean_error = sum_error / static_cast<double>(tracked);
  const double track_fraction = std::clamp(
    static_cast<double>(tracked) / static_cast<double>(max_features_), 0.0, 1.0);
  const double error_quality = 1.0 / (1.0 + mean_error / 10.0);
  const double quality = std::clamp(track_fraction * error_quality, 0.0, 1.0);

  VisualOdometryEstimate estimate;
  estimate.velocity_estimate = {
    mean_vx_m / dt_s,
    mean_vy_m / dt_s,
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

double SparseOpticalFlowVO::estimate_depth_scale_at(
  const cv::Mat & depth_frame,
  const cv::Point2f & point) const
{
  if (depth_frame.empty()) {
    return fallback_m_per_pixel_;
  }

  const int x = std::clamp(static_cast<int>(std::lround(point.x)), 0, depth_frame.cols - 1);
  const int y = std::clamp(static_cast<int>(std::lround(point.y)), 0, depth_frame.rows - 1);

  double depth_m = 0.0;
  if (depth_frame.type() == CV_32FC1) {
    depth_m = static_cast<double>(depth_frame.at<float>(y, x));
  } else if (depth_frame.type() == CV_16UC1) {
    depth_m = static_cast<double>(depth_frame.at<uint16_t>(y, x)) * 0.001;
  } else {
    return fallback_m_per_pixel_;
  }

  if (!std::isfinite(depth_m) || depth_m <= 0.05) {
    return fallback_m_per_pixel_;
  }
  return std::clamp(depth_m / focal_length_px_, 1e-4, 0.1);
}

}  // namespace twinguard::estimation
