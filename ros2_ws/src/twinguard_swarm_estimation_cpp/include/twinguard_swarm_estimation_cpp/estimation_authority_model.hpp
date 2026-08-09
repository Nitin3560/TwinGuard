#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace twinguard::estimation
{

enum SensorMask : std::uint32_t
{
  SENSOR_PX4_ODOMETRY = 1u << 0,
  SENSOR_VISUAL_ODOMETRY = 1u << 1,
  SENSOR_GPS = 1u << 2,
  SENSOR_IMU = 1u << 3,
};

struct EstimationAuthorityInput
{
  std::array<double, 3> position_variance{0.0, 0.0, 0.0};
  double position_nis{0.0};
  double velocity_nis{0.0};
  double measurement_age_ms{0.0};
  std::uint32_t active_sensor_mask{0u};
  std::uint32_t required_sensor_mask{SENSOR_PX4_ODOMETRY};
};

struct EstimationAuthorityConfig
{
  double max_position_sigma_m{1.0};
  double nis_limit{10.0};
  double stale_timeout_ms{500.0};
};

struct EstimationAuthorityResult
{
  double covariance_factor{1.0};
  double nis_factor{1.0};
  double freshness_factor{1.0};
  double availability_factor{1.0};
  double estimation_factor{1.0};
  std::string estimation_limiting_reason{"none"};
  double position_sigma_m{0.0};
  double position_nis{0.0};
  double velocity_nis{0.0};
  double measurement_age_ms{0.0};
  std::uint32_t active_sensor_mask{0u};
};

class EstimationAuthorityModel
{
public:
  explicit EstimationAuthorityModel(EstimationAuthorityConfig config = {});

  EstimationAuthorityResult evaluate(const EstimationAuthorityInput & input) const;

private:
  static double clamp01(double value);
  static int popcount(std::uint32_t value);

  EstimationAuthorityConfig config_;
};

}  // namespace twinguard::estimation
