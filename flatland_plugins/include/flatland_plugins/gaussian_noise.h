#ifndef FLATLAND_PLUGINS_GAUSSIAN_NOISE_H
#define FLATLAND_PLUGINS_GAUSSIAN_NOISE_H

#include <random>

namespace flatland_plugins
{

/// std::normal_distribution requires stddev > 0; this returns the mean when stddev <= 0
class GaussianNoise
{
public:
  GaussianNoise() = default;
  GaussianNoise(double mean, double stddev)
  : dist_(mean, stddev > 0.0 ? stddev : 1.0), mean_(mean), enabled_(stddev > 0.0)
  {
  }

  template <class Generator>
  double operator()(Generator & g)
  {
    return enabled_ ? dist_(g) : mean_;
  }

private:
  std::normal_distribution<double> dist_;
  double mean_ = 0.0;
  bool enabled_ = false;
};

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_GAUSSIAN_NOISE_H
