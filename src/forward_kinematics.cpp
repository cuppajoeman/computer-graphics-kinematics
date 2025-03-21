#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include <functional> // std::function

void forward_kinematics(
  const Skeleton & skeleton,
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T)
{
  T.resize(skeleton.size(),Eigen::Affine3d::Identity());
  for (int i = 0; i < skeleton.size(); i++){

    // Note that parent is a mapping where it takes in a vertex
    // in rest space, brings it to local model space, then applies
    // a transform and then maps it back into rest space
    // thus it is of the form (RP) Rot (RP)^{-1}
    // then also due to the fact that we need to recursively do the transform 
    // up the bone structure it becomes of the form P (RP) Rot (RP)^{-1}
    // where P is a transform of the same type (recursive)
    Eigen::Affine3d parent = Eigen::Affine3d::Identity();
    
    auto &curr_bone = skeleton[i];
    bool bone_is_root = curr_bone.parent_index == -1;
    if (not bone_is_root) {
      parent = T[curr_bone.parent_index];
    }
    
    T[i] = parent * curr_bone.rest_T * euler_angles_to_transform(curr_bone.xzx) * (curr_bone.rest_T).inverse();
  }
}
