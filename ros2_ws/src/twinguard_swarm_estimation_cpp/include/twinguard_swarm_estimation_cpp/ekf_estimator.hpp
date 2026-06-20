#pragma once

#include <array>

#include <Eigen/Dense>

namespace twinguard::estimation
{

struct EkfState
{
  Eigen::Matrix<double, 6, 1> x{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 6> P{Eigen::Matrix<double, 6, 6>::Identity()};
  double nis{0.0};
};

class EkfEstimator
{
public:
  explicit EkfEstimator(double process_noise_std = 0.5);

  void reset(
    const std::array<double, 3> & position,
    const std::array<double, 3> & velocity);

  void predict(double dt_s);

  double update_position(
    const std::array<double, 3> & measured_position,
    double measurement_noise_std);

  double update_velocity(
    const std::array<double, 3> & measured_velocity,
    double measurement_noise_std);

  const EkfState & state() const { return state_; }

private:
  double update(
    const Eigen::Vector3d & z,
    const Eigen::Matrix<double, 3, 6> & H,
    double measurement_noise_std);

  EkfState state_;
  double process_noise_std_;
};

}  // namespace twinguard::estimation
