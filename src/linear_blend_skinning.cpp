#include "linear_blend_skinning.h"

void linear_blend_skinning(
  const Eigen::MatrixXd & V,
  const Skeleton & skeleton,
  const std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T,
  const Eigen::MatrixXd & W,
  Eigen::MatrixXd & U)
{
  U.resize(V.rows(), 3);

  for(int i = 0; i < V.rows(); i++){
    Eigen::RowVector3d rest_pose_pos = V.row(i);
    Eigen::Vector3d bone_affected_pos = Eigen::Vector3d::Zero();
    
    for(int j = 0; j < skeleton.size(); j++){
      Bone curr_bone = skeleton[j];
      if (curr_bone.weight_index != -1){
        // following the specfied equation
        auto curr_bone_weight_on_vertex = W(i, curr_bone.weight_index);
        bone_affected_pos +=  curr_bone_weight_on_vertex * (T[j] * rest_pose_pos.transpose()).transpose();
      }
    }
    U.row(i) = bone_affected_pos;
  }
}
