#include "twinguard_swarm_estimation_cpp/ekf_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace twinguard::estimation
{

EkfEstimator::EkfEstimator(double process_noise_std)
: process_noise_std_(std::max(process_noise_std, 1e-6))
{
}

void EkfEstimator::reset(
  const std::array<double, 3> & position,
  const std::array<double, 3> & velocity)
{
  state_.x <<
    position[0], position[1], position[2],
    velocity[0], velocity[1], velocity[2];
  state_.P = Eigen::Matrix<double, 6, 6>::Identity();
  state_.nis = 0.0;
}

void EkfEstimator::predict(double dt_s)
{
  const double dt = std::max(dt_s, 1e-4);
  Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
  F(0, 3) = dt;
  F(1, 4) = dt;
  F(2, 5) = dt;

  const double q = process_noise_std_ * process_noise_std_;
  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
  Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * q * dt * dt;
  Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * q * dt;

  state_.x = F * state_.x;
  state_.P = F * state_.P * F.transpose() + Q;
}

double EkfEstimator::update_position(
  const std::array<double, 3> & measured_position,
  double measurement_noise_std)
{
  Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  H(2, 2) = 1.0;
  return update(Eigen::Vector3d(measured_position[0], measured_position[1], measured_position[2]), H,
    measurement_noise_std);
}

double EkfEstimator::update_velocity(
  const std::array<double, 3> & measured_velocity,
  double measurement_noise_std)
{
  Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
  H(0, 3) = 1.0;
  H(1, 4) = 1.0;
  H(2, 5) = 1.0;
  return update(Eigen::Vector3d(measured_velocity[0], measured_velocity[1], measured_velocity[2]), H,
    measurement_noise_std);
}

double EkfEstimator::update(
  const Eigen::Vector3d & z,
  const Eigen::Matrix<double, 3, 6> & H,
  double measurement_noise_std)
{
  const double stddev = std::max(measurement_noise_std, 1e-6);
  const Eigen::Vector3d innovation = z - H * state_.x;
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * stddev * stddev;
  const Eigen::Matrix3d S = H * state_.P * H.transpose() + R;
  const Eigen::Matrix<double, 6, 3> K = state_.P * H.transpose() * S.inverse();
  const Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();

  state_.x = state_.x + K * innovation;
  state_.P = (I - K * H) * state_.P * (I - K * H).transpose() + K * R * K.transpose();
  state_.nis = innovation.transpose() * S.inverse() * innovation;
  return state_.nis;
}

}  // namespace twinguard::estimation
