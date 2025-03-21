#include "projected_gradient_descent.h"
#include "line_search.h"

void projected_gradient_descent(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const int max_iters,
  Eigen::VectorXd & z)
{
  for (int i = 0; i < max_iters; i++){
    // df(z) / dz
    Eigen::VectorXd grad_f_z = grad_f(z);

    // I'm using 10000 here because it was specfied in the 
    // readme, and seems to work alright, so not changing it.
    int max_iters = 10000;
    double alpha = line_search(f, proj_z, z, grad_f_z, max_iters);
    z = z - alpha * grad_f_z;
    proj_z(z);
  }
}
