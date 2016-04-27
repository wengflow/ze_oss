#pragma once

#include <vector>
#include <memory>
#include <ze/common/logging.hpp>
#include <Eigen/Core>

#include <ze/common/types.h>
#include <ze/common/statistics.h>

namespace ze {

// -----------------------------------------------------------------------------
// Scale Estimators

//! Dummy scale estimator.
template <typename Scalar>
struct UnitScaleEstimator
{
  static constexpr Scalar compute(const VectorX& /*errors*/)
  {
    return Scalar{1.0};
  }
};

//! Estimates scale by computing the median absolute deviation (MAD).
template <typename Scalar>
struct MADScaleEstimator
{
  using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  static Scalar compute(const VectorX& errors)
  {
    VectorX absolute_error(errors.size());
    absolute_error = errors.array().abs();
    std::pair<Scalar, bool> res = median(absolute_error);
    CHECK(res.second);
    return Scalar{1.48} * res.first; // 1.48f / 0.6745
  }
};

//! Estimates scale by computing the standard deviation.
template <typename Scalar>
struct NormalDistributionScaleEstimator
{
  using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  static Scalar compute(const VectorX& errors)
  {
    // normed_errors should not have absolute values.
    const int n = errors.size();
    CHECK(n > 1);
    const Scalar mean = errors.sum() / n;
    Scalar sum2 = Scalar{0.0};
    for(int i = 0; i < errors.size(); ++i)
    {
      sum2 += (errors(i) - mean) * (errors(i) - mean);
    }
    return std::sqrt(sum2 / (n - 1));
  }
};


// -----------------------------------------------------------------------------
// Scale Estimators
// Weight-Functions for M-Estimators
// http://research.microsoft.com/en-us/um/people/zhang/inria/publis/tutorial-estim/node24.html

template <typename Scalar, typename Implementation>
struct WeightFunction
{
  using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  static VectorX weightVectorized(const VectorX& error_vec)
  {
    VectorX weights(error_vec.size());
    for(int i = 0; i < error_vec.size(); ++i)
    {
      weights(i) = Implementation::weight(error_vec(i));
    }
    return weights;
  }

  static FloatType weight(const FloatType error)
  {
    return Implementation::weight(error);
  }
};

template <typename Scalar>
struct UnitWeightFunction : public WeightFunction<Scalar, UnitWeightFunction<Scalar>>
{
  static constexpr Scalar weight(const Scalar& /*normed_error*/)
  {
    return Scalar{1.0};
  }
};

template <typename Scalar>
struct TukeyWeightFunction : public WeightFunction<Scalar, TukeyWeightFunction<Scalar>>
{
  static constexpr Scalar b_square = 4.6851 * 4.6851;
  static Scalar weight(const Scalar& error)
  {
    const Scalar x_square = error * error;
    if(x_square <= b_square)
    {
      const Scalar tmp = Scalar{1.0} - x_square / b_square;
      return tmp * tmp;
    }
    else
    {
      return Scalar{0.0};
    }
  }
};

template <typename Scalar>
struct HuberWeightFunction : public WeightFunction<Scalar, HuberWeightFunction<Scalar>>
{
  static constexpr Scalar k = 1.345;
  static Scalar weight(const Scalar& normed_error)
  {
    const Scalar abs_error = std::abs(normed_error);
    return (abs_error < k) ? Scalar{1.0} : k / abs_error;
  }
};

} // namespace ze
