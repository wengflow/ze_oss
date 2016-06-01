#pragma once

// The below code and structure is heavily inspired by gtsam's
// navigation/Scenario.h and ScenarioRunner.h

#include <ze/imu/scenario.h>
#include <ze/imu/imu_bias.h>
#include <ze/common/sampler.h>

namespace ze {

//! given a trajectory scenario generates imu measurements
class ScenarioRunner
{
public:
  //! construct a running given a scenario itself
  ScenarioRunner(const Scenario::Ptr scenario,
                 const ImuBias::Ptr bias,
                 GaussianSampler<3>::Ptr accelerometer_noise,
                 GaussianSampler<3>::Ptr gyro_noise,
                 const Vector3& gravity,
                 double imu_sample_time = 1.0 / 100.0)
      : scenario_(scenario)
      , bias_(bias)
      , accelerometer_noise_(accelerometer_noise)
      , gyro_noise_(gyro_noise)
      , gravity_(gravity)
      , imu_sample_time_(imu_sample_time)
      , sqrt_dt_(std::sqrt(imu_sample_time))
  {}

  //! Gravity fixed along Z axis
  const Vector3& gravity() const
  {
    return gravity_;
  }

  //! angular velocity in body frame
  Vector3 angular_velocity_actual(double t) const
  {
    return scenario_->angular_velocity_body(t);
  }

  //! An accelerometer measures acceleration in body, but not gravity
  Vector3 acceleration_actual(double t) const
  {
    Matrix3 Rbn(scenario_->orientation(t).transpose());
    return scenario_->acceleration_body(t) - Rbn * gravity();
  }

  // versions corrupted by bias and noise
  Vector3 angular_velocity_corrupted(double t) const
  {
    return angular_velocity_actual(t) + bias_->gyroscope(t) +
           gyro_noise_->sample() / sqrt_dt_;
  }

  Vector3 acceleration_corrupted(double t) const
  {
    return acceleration_actual(t) + bias_->accelerometer(t) +
           accelerometer_noise_->sample() / sqrt_dt_;
  }

  const double& imu_sample_time() const
  {
    return imu_sample_time_;
  }

private:
  const Scenario::Ptr scenario_;
  const ImuBias::Ptr bias_;

  // Create two samplers for acceleration and omega noise
  mutable GaussianSampler<3>::Ptr accelerometer_noise_;
  mutable GaussianSampler<3>::Ptr gyro_noise_;

  const Vector3& gravity_;

  const FloatType imu_sample_time_;
  const FloatType sqrt_dt_;
};

} // namespace ze
