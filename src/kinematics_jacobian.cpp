#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>

void kinematics_jacobian(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  Eigen::MatrixXd & J)
{
  // first we zero out the matrix.
  J = Eigen::MatrixXd::Zero(b.size() * 3, skeleton.size() * 3);
  Eigen::VectorXd transformed_bone_tips = transformed_tips(skeleton, b);
  // I tried smaller and smaller exponents, but after e-8 it stopped functioning correctly
  // so leaving it at e-7
  double epsilon = 10e-7;

  // for each bone
  for (int i = 0; i < skeleton.size(); i++) {
    // for each rotation axis
    for (int j = 0; j < 3; j++){
      Skeleton finite_difference_skel = skeleton;
      finite_difference_skel[i].xzx[j] += epsilon;
      // finite difference along one rotation component
      J.col(3*i + j) = (transformed_tips(finite_difference_skel,b) - transformed_bone_tips) / epsilon;
    }
  }
}
