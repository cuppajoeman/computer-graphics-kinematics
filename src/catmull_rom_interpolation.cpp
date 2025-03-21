#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d,
           double, double, double, double>
get_control_points_and_times(double t, const std::vector<std::pair<double, Eigen::Vector3d>>& keyframes) {
    if (keyframes.empty()) {
        throw std::invalid_argument("Keyframes cannot be empty.");
    }

    // locate the next keyframe index for the current time
    int next_keyframe_index = keyframes.size() - 1;
    for (int i = 0; i < keyframes.size(); i++) {
        if (keyframes[i].first > t) {
            next_keyframe_index = i;
            break;
        }
    }


    /*
    *                     |                        
    *                     |   cp2
    *                     |                        
    *    cp0              |                        
    *                     |                        
    *                     |                        
    *                     |             cp3
    *           cp1       |                        
    *                     |                        
    *                     |                        
    *                     t                        
    */
    Eigen::Vector3d control_point_0, control_point_1, control_point_2, control_point_3;
    double time_0, time_1, time_2, time_3;

    int num_keyframes = keyframes.size();

    // safe indexing
    int index_0 = (next_keyframe_index - 2 + num_keyframes) % num_keyframes;
    int index_1 = (next_keyframe_index - 1 + num_keyframes) % num_keyframes;
    int index_3 = (next_keyframe_index + 1) % num_keyframes;

    control_point_0 = keyframes[index_0].second;
    control_point_1 = keyframes[index_1].second;
    control_point_2 = keyframes[next_keyframe_index].second; // Current keyframe
    control_point_3 = keyframes[index_3].second;

    time_0 = keyframes[index_0].first;
    time_1 = keyframes[index_1].first;
    time_2 = keyframes[next_keyframe_index].first; // Current keyframe
    time_3 = keyframes[index_3].first;

    return {control_point_0, control_point_1, control_point_2, control_point_3,
            time_0, time_1, time_2, time_3};
}

Eigen::Vector3d catmull_rom_interpolation(
  const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
  double t)
{

  if (keyframes.empty()) {
      return Eigen::Vector3d(0, 0, 0);
  }

  // we wrap the time based on the last possible keyframe time so that the animation loops
  double last_keyframe_time = keyframes.back().first;
  t = std::fmod(t, last_keyframe_time);

  auto [control_point_0, control_point_1, control_point_2, control_point_3, time_0, time_1, time_2, time_3] = get_control_points_and_times(t, keyframes);

  const auto tangent = [](
    const Eigen::Vector3d p1,
    const  Eigen::Vector3d p2, double t1, double t2){
    return (p2 - p1) / (t2 - t1);
  };

  double t1 = (t - time_1) / (time_2 - time_1);
  // doing powers
  double t2 = t1 * t1;
  double t3 = t2 * t1;

  Eigen::Vector3d C;
  C = (2 * t3 - 3 * t2 + 1) * control_point_1 + 
           (- 2 * t3 + 3 * t2 ) * control_point_2 + 
        (t3 - 2 * t2 + t1) * tangent(control_point_0, control_point_2, time_0, time_2) + 
              (t3 - t2) * tangent(control_point_1, control_point_3, time_1, time_3);


  return C;

}
