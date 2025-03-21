#include "euler_angles_to_transform.h"

Eigen::Affine3d euler_angles_to_transform(
  const Eigen::Vector3d & xzx)
{
  // we will rotate on each axis, then multiply all the resulting matrices
  Eigen::Affine3d x_rotation_mat_1, z_rotation_mat_2, x_rotation_mat_2;

  double x_rot_1_rads =  xzx[0] / 180.0 * M_PI;
  double y_rot_rads =  xzx[1] / 180.0 * M_PI;
  double gammax_rot_2_rads =  xzx[2] / 180.0 * M_PI;

  x_rotation_mat_1.matrix() << 
    1, 0, 0, 0, 
    0, cos(x_rot_1_rads), -sin(x_rot_1_rads), 0, 
    0, sin(x_rot_1_rads), cos(x_rot_1_rads), 0, 
    0, 0, 0, 1;
  z_rotation_mat_2.matrix() << 
    cos(y_rot_rads), -sin(y_rot_rads), 0, 0, 
    sin(y_rot_rads), cos(y_rot_rads), 0, 0, 
    0, 0, 1, 0, 
    0, 0, 0, 1;
  x_rotation_mat_2.matrix() << 
    1, 0, 0, 0, 
    0, cos(gammax_rot_2_rads), -sin(gammax_rot_2_rads), 0, 
    0, sin(gammax_rot_2_rads), cos(gammax_rot_2_rads), 0, 
    0, 0, 0, 1;

  return x_rotation_mat_2 * z_rotation_mat_2 * x_rotation_mat_1;
}
