#include "transformed_tips.h"
#include "forward_kinematics.h"

Eigen::VectorXd transformed_tips(
  const Skeleton & skeleton, 
  const Eigen::VectorXi & b)
{
  Eigen::VectorXd bone_tip_positions = Eigen::VectorXd::Zero(3*b.size());

  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > T;
  forward_kinematics(skeleton, T);

  for (int i = 0; i < b.size(); i++) {
    int bone_index = b[i];
    Bone curr_bone = skeleton[bone_index];

    auto fk_transform = T[bone_index];
    Eigen::Vector4d homog_tip_pos(curr_bone.length, 0, 0, 1);
    // take the tip, move it into rest pose, and then apply the transform
    // the transform is coming from the animation process where we interp
    // the transform between keyframes
    Eigen::VectorXd transformed_tip =  fk_transform * curr_bone.rest_T * homog_tip_pos;

    // copy over positions
    bone_tip_positions[3 * i] = transformed_tip[0];
    bone_tip_positions[3 * i + 1] = transformed_tip[1];
    bone_tip_positions[3 * i + 2] = transformed_tip[2];
  }

  return bone_tip_positions;
}
