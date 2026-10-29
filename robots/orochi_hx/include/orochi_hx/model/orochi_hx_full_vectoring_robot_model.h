// -*- mode: c++ -*-

#pragma once

#include <ros/ros.h>

#include <aerial_robot_model/model/aerial_robot_model.h>
#include <mutex>

using namespace aerial_robot_model;

/*
  Robot model for the full vectoring mode of orochi_hx (see manual_for_full_vectoring.md)

  - the "link" of dragon corresponds to female{i} of orochi_hx
  - 2DoF joint (roll, yaw):
      phi_j = phi^m + phi^f  (male_roll{i} + female_roll{i}, between male{i} and female{i})
      psi_j = psi^j          (joint_yaw{i}, between female{i} and male{i+1})
  - 2DoF gimbal (roll, pitch):
      phi_g = phi^f          (female_roll{i})
      th_g  = th^g           (gimbal_pitch{i})
    the thrust direction expressed in the female{i} frame is Rx(-phi_g) * Ry(th_g) * e_z
*/
class OrochiHxFullVectoringRobotModel : public aerial_robot_model::RobotModel
{
public:
  OrochiHxFullVectoringRobotModel(bool init_with_rosparam = true, bool verbose = false, double fc_f_min_thre = 0,
                                  double fc_t_min_thre = 0, double epsilon = 10);
  ~OrochiHxFullVectoringRobotModel() = default;

  template <class T>
  std::vector<T> getLinksRotationFromCog();

  const std::vector<double> getCurrentGimbalRollAngles()
  {
    std::lock_guard<std::mutex> lock(current_angles_mutex_);
    return current_gimbal_roll_angles_;
  }
  const std::vector<double> getCurrentGimbalPitchAngles()
  {
    std::lock_guard<std::mutex> lock(current_angles_mutex_);
    return current_gimbal_pitch_angles_;
  }
  const std::vector<double> getCurrentJointRollAngles()
  {
    std::lock_guard<std::mutex> lock(current_angles_mutex_);
    return current_joint_roll_angles_;
  }
  const std::vector<double> getCurrentJointYawAngles()
  {
    std::lock_guard<std::mutex> lock(current_angles_mutex_);
    return current_joint_yaw_angles_;
  }

private:
  std::mutex links_rotation_mutex_;
  std::mutex current_angles_mutex_;

  std::vector<KDL::Rotation> links_rotation_from_cog_;  // female{i} w.r.t. cog
  std::vector<double> current_gimbal_roll_angles_;      // phi_g = phi^f
  std::vector<double> current_gimbal_pitch_angles_;     // th_g = th^g
  std::vector<double> current_joint_roll_angles_;       // phi_j = phi^m + phi^f
  std::vector<double> current_joint_yaw_angles_;        // psi_j = psi^j

protected:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
};

template <>
inline std::vector<KDL::Rotation> OrochiHxFullVectoringRobotModel::getLinksRotationFromCog()
{
  std::lock_guard<std::mutex> lock(links_rotation_mutex_);
  return links_rotation_from_cog_;
}

template <>
inline std::vector<Eigen::Matrix3d> OrochiHxFullVectoringRobotModel::getLinksRotationFromCog()
{
  return aerial_robot_model::kdlToEigen(getLinksRotationFromCog<KDL::Rotation>());
}
