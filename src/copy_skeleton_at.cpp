#include "copy_skeleton_at.h"
Skeleton copy_skeleton_at(
  const Skeleton & skeleton, 
  const Eigen::VectorXd & A)
{
  Skeleton skel_copy = skeleton;
  for (int i = 0; i < skeleton.size(); i++){
    auto alpha = A[3 * i];
    auto beta = A[3 * i + 1];
    auto gamma = A[3 * i + 2];
    skel_copy[i].xzx = Eigen::Vector3d(alpha, beta, gamma);
  }
  return skel_copy;
}
