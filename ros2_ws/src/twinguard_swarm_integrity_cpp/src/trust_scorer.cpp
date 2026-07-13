#include "twinguard_swarm_integrity_cpp/trust_scorer.hpp"

#include <algorithm>
#include <cmath>

namespace twinguard::integrity
{

namespace
{

double residual_norm(
  const std::array<double, 3> & measured,
  const std::array<double, 3> & predicted)
{
  const double dx = measured[0] - predicted[0];
  const double dy = measured[1] - predicted[1];
  const double dz = measured[2] - predicted[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::string classify_fault(double residual, double trust)
{
  if (trust < 0.35 && residual > 1.0) {
    return "suspected_attack";
  }
  if (trust < 0.65) {
    return "degraded";
  }
  return "nominal";
}

}  // namespace

TrustScorer::TrustScorer(double alpha, double beta, double min_authority)
: alpha_(alpha),
  beta_(beta),
  min_authority_(min_authority)
{
}

TrustState TrustScorer::update(
  const std::array<double, 3> & measured_position,
  const std::array<double, 3> & predicted_position)
{
  const double residual = residual_norm(measured_position, predicted_position);
  const double integrity = std::exp(-alpha_ * residual);
  const double trust = std::clamp(beta_ * state_.trust + (1.0 - beta_) * integrity, 0.0, 1.0);
  const double authority = std::clamp(trust, min_authority_, 1.0);

  state_ = TrustState{
    trust,
    residual,
    authority,
    classify_fault(residual, trust),
  };
  return state_;
}

const TrustState & TrustScorer::state() const
{
  return state_;
}

DigitalTwinPredictor::DigitalTwinPredictor(double dt)
: dt_(dt)
{
}

void DigitalTwinPredictor::reset(
  const std::array<double, 3> & position,
  const std::array<double, 3> & velocity)
{
  position_ = position;
  velocity_ = velocity;
  initialized_ = true;
}

std::array<double, 3> DigitalTwinPredictor::predict()
{
  if (initialized_) {
    position_[0] += velocity_[0] * dt_;
    position_[1] += velocity_[1] * dt_;
    position_[2] += velocity_[2] * dt_;
  }
  return position_;
}

void DigitalTwinPredictor::correct_velocity(
  const std::array<double, 3> & velocity,
  double alpha)
{
  velocity_[0] = (1.0 - alpha) * velocity_[0] + alpha * velocity[0];
  velocity_[1] = (1.0 - alpha) * velocity_[1] + alpha * velocity[1];
  velocity_[2] = (1.0 - alpha) * velocity_[2] + alpha * velocity[2];
}

void DigitalTwinPredictor::correct_position(
  const std::array<double, 3> & position,
  double weight)
{
  const double gain = std::clamp(weight, 0.0, 1.0);
  position_[0] += gain * (position[0] - position_[0]);
  position_[1] += gain * (position[1] - position_[1]);
  position_[2] += gain * (position[2] - position_[2]);
}

}  // namespace twinguard::integrity
