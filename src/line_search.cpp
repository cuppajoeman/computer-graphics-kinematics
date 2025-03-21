#include "line_search.h"
#include <iostream>

double line_search(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & dz,
  const double max_step)
{
  double alpha = max_step;

  Eigen::VectorXd z_gradient_step = z - alpha * dz;
  proj_z(z_gradient_step);

  // here, our goal is to find a good value for alpha, such that 
  // the place where we step is smaller than our initial position.
  // also note that by (A) this while loop will terminate.
  while (f(z_gradient_step) > f(z)) {
    alpha *= 0.5;
    z_gradient_step = z - alpha * dz;

    // make sure where we stepped is safe.
    proj_z(z_gradient_step);

    // (A) this occurs when alpha has been multiplied by itself enough times
    // until it becomes zero as floating point can no longer represent
    // how small the value is thus one the above line when we set 
    // z_gradient_step = z - alpha * dz we readlly did z_gradient_step = z
    // explaining the following line here:
    if (z_gradient_step == z){
      // because alpha is zero.
      return 0;
    }

  }
  return alpha;
}
