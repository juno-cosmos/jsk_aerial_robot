#include <orochi_hx/model/orochi_hx_full_vectoring_robot_model.h>

OrochiHxFullVectoringRobotModel::OrochiHxFullVectoringRobotModel(bool init_with_rosparam, bool verbose,
                                                                 double fc_f_min_thre, double fc_t_min_thre,
                                                                 double epsilon)
  : RobotModel(init_with_rosparam, verbose, false /* non-fixed model */, fc_f_min_thre, fc_t_min_thre, epsilon)
{
  const int rotor_num = getRotorNum();
  links_rotation_from_cog_.resize(rotor_num);
  current_gimbal_roll_angles_.resize(rotor_num);
  current_gimbal_pitch_angles_.resize(rotor_num);
  current_joint_roll_angles_.resize(rotor_num);
  current_joint_yaw_angles_.resize(rotor_num - 1);  // joint_yaw1..joint_yaw(N-1)
}

void OrochiHxFullVectoringRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  RobotModel::updateRobotModelImpl(joint_positions);

  const auto& seg_tf_map = getSegmentsTf();
  const auto& joint_index_map = getJointIndexMap();
  KDL::Frame cog = getCog<KDL::Frame>();

  const int rotor_num = getRotorNum();
  std::vector<KDL::Rotation> links_rotation_from_cog(rotor_num);
  std::vector<double> gimbal_roll(rotor_num), gimbal_pitch(rotor_num), joint_roll(rotor_num);
  std::vector<double> joint_yaw(rotor_num - 1);

  for (int i = 0; i < rotor_num; ++i)
  {
    std::string s = std::to_string(i + 1);

    /* link: female{i} corresponds to the link of dragon */
    links_rotation_from_cog.at(i) = (cog.Inverse() * seg_tf_map.at("female" + s)).M;

    double male_roll = joint_positions(joint_index_map.at("male_roll" + s));
    double female_roll = joint_positions(joint_index_map.at("female_roll" + s));
    gimbal_roll.at(i) = female_roll;
    gimbal_pitch.at(i) = joint_positions(joint_index_map.at("gimbal_pitch" + s));
    joint_roll.at(i) = male_roll + female_roll;

    if (i < rotor_num - 1)
      joint_yaw.at(i) = joint_positions(joint_index_map.at("joint_yaw" + s));
  }

  {
    std::lock_guard<std::mutex> lock(links_rotation_mutex_);
    links_rotation_from_cog_ = links_rotation_from_cog;
  }
  {
    std::lock_guard<std::mutex> lock(current_angles_mutex_);
    current_gimbal_roll_angles_ = gimbal_roll;
    current_gimbal_pitch_angles_ = gimbal_pitch;
    current_joint_roll_angles_ = joint_roll;
    current_joint_yaw_angles_ = joint_yaw;
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(OrochiHxFullVectoringRobotModel, aerial_robot_model::RobotModel);
