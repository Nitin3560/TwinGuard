#pragma once

#include <array>
#include <string>

namespace twinguard::integrity
{

struct TrustState
{
  double trust{1.0};
  double residual{0.0};
  double authority_scale{1.0};
  std::string fault_label{"nominal"};
};

class TrustScorer
{
public:
  TrustScorer(double alpha = 1.2, double beta = 0.90, double min_authority = 0.15);

  TrustState update(
    const std::array<double, 3> & measured_position,
    const std::array<double, 3> & predicted_position);

  const TrustState & state() const;

private:
  double alpha_;
  double beta_;
  double min_authority_;
  TrustState state_;
};

class DigitalTwinPredictor
{
public:
  explicit DigitalTwinPredictor(double dt = 0.02);

  void reset(
    const std::array<double, 3> & position,
    const std::array<double, 3> & velocity = {0.0, 0.0, 0.0});

  std::array<double, 3> predict();

  void correct_velocity(const std::array<double, 3> & velocity, double alpha = 0.2);
  void correct_position(const std::array<double, 3> & position, double weight);

private:
  double dt_;
  bool initialized_{false};
  std::array<double, 3> position_{0.0, 0.0, 0.0};
  std::array<double, 3> velocity_{0.0, 0.0, 0.0};
};

}  // namespace twinguard::integrity
