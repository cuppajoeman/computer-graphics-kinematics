#include "end_effectors_objective_and_gradient.h"
#include "transformed_tips.h"
#include "kinematics_jacobian.h"
#include "copy_skeleton_at.h"
#include <iostream>

void end_effectors_objective_and_gradient(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  const Eigen::VectorXd & xb0,
  std::function<double(const Eigen::VectorXd &)> & f,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  std::function<void(Eigen::VectorXd &)> & proj_z)
{
  f = [&](const Eigen::VectorXd & A)->double
  {
    // compute the objective function:
    Skeleton skel_copy = copy_skeleton_at(skeleton, A);
    Eigen::VectorXd transformed_bone_tips = transformed_tips(skel_copy, b);
    return (transformed_bone_tips - xb0).dot(transformed_bone_tips - xb0);
  };
  grad_f = [&](const Eigen::VectorXd & A)->Eigen::VectorXd
  {
    Skeleton skel_copy = copy_skeleton_at(skeleton, A);
    Eigen::VectorXd transformed_bone_tips = transformed_tips(skel_copy, b);
    Eigen::MatrixXd jacobian;
    kinematics_jacobian(skel_copy, b, jacobian);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(A.size());
    // df(theta) / d theta = [d ( || x(theta)  - q ||^2_2 )] / [d theta]
    // now use the chain rule = 2 [d x(theta)] / [d theta] (x (theta) - q)
    // and note that [d x(theta)] / [d theta] we use the jacobian.
    gradient =  2 * jacobian.transpose() *  (transformed_bone_tips - xb0);
    return gradient;
  };
  proj_z = [&](Eigen::VectorXd & A)
  {
    for (int i=0; i < skeleton.size(); i++){
      auto curr_bone = skeleton[i];
      // here we take the current state and clamp the angles into the admissiable positions
      // so that un-natural configurations of the model cannot occur.
      A[3 * i] = std::fmax(curr_bone.xzx_min[0], std::fmin(curr_bone.xzx_max[0], A[3 * i]));
      A[3 * i + 1] = std::fmax(curr_bone.xzx_min[1], std::fmin(curr_bone.xzx_max[1], A[3 * i + 1]));
      A[3 * i + 2] = std::fmax(curr_bone.xzx_min[2], std::fmin(curr_bone.xzx_max[2], A[3 * i + 2]));
    }
    assert(skeleton.size()*3 == A.size());
  };
}
