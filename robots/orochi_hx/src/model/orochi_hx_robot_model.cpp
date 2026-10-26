#include <orochi_hx/model/orochi_hx_robot_model.h>

OrochiHxRobotModel::OrochiHxRobotModel(bool init_with_rosparam, bool verbose, double fc_f_min_thre, double fc_t_min_thre, double epsilon) : RobotModel(init_with_rosparam, verbose, false /* non-fixed model*/, fc_f_min_thre, fc_t_min_thre, epsilon)
{
  const int rotor_num = getRotorNum();
  links_rotation_from_cog_.resize(rotor_num);
  current_joint_angles_.resize(getJointNum() - rotor_num); // 15 - 4 = 11: male_roll x4, female_roll x4, joint_yaw x3
  current_gimbal_angles_.resize(rotor_num); // gimbal_pitch x4
}

void OrochiHxRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  auto start = std::chrono::high_resolution_clock::now();

  RobotModel::updateRobotModelImpl(joint_positions);

  const auto& seg_tf_map = getSegmentsTf();
  const auto& joint_index_map = getJointIndexMap();

  KDL::Frame cog = getCog<KDL::Frame>();

  /* link and rotor information in each link */
  for (int i = 0; i < getRotorNum(); ++i)
  {
    std::string s = std::to_string(i + 1);

    /* link */
    KDL::Frame link_f = seg_tf_map.at("body" + s);
    links_rotation_from_cog_.at(i) = (cog.Inverse() * link_f).M;

    /* get current gimbal angles */
    current_gimbal_angles_.at(i) = joint_positions(joint_index_map.at("gimbal_pitch" + s));

  }

  /* get current joint angles (all non-gimbal joints) */
  const auto& joint_names = getJointNames();
  const auto& joint_indices = getJointIndices();
  int j = 0;
  for (size_t k = 0; k < joint_names.size(); k++)
  {
    if (joint_names.at(k).find("gimbal") == 0) // rotor以外のjointからgimbalが入っている名前のjointを抜く
      continue;
    current_joint_angles_.at(j++) = joint_positions(joint_indices.at(k));
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
}

Eigen::MatrixXd OrochiHxRobotModel::getFullWrenchAllocationMatrixFromCog()
{
  /* calculate normal allocation */
  const int rotor_num = getRotorNum();
  Eigen::MatrixXd wrench_matrix = Eigen::MatrixXd::Zero(6, 3 * rotor_num);
  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

  /* get cog */
  KDL::Frame cog = getCog<KDL::Frame>();

  int last_col = 0;
  std::vector<KDL::Vector> rotors_origin_from_cog = getRotorsOriginFromCog<KDL::Vector>();
  double m_f_rate = getMFRate();
  std::map<int, int> rotor_direction = getRotorDirection();
  for (int i = 0; i < rotor_num; i++)
  {
    wrench_map.block(3, 0, 3, 3) =
        aerial_robot_model::skew(kdlToEigen(rotors_origin_from_cog.at(i))) +
        m_f_rate * rotor_direction.at(i + 1) * Eigen::MatrixXd::Identity(3, 3);  // [p x] + sigma E_3
    wrench_matrix.middleCols(last_col, 3) = wrench_map;
    last_col += 3;
  }

  /* calculate masked and integrated rotaion matrix */
  Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * rotor_num, 2 * rotor_num);
  Eigen::MatrixXd mask(3, 2);  // x and z axis
  mask << 1, 0,
          0, 0,
          0, 1;
  for (int i = 0; i < rotor_num; i++)
  {
    integrated_rot.block(3 * i, 2 * i, 3, 2) = kdlToEigen(links_rotation_from_cog_.at(i)) * mask;
  }

  /* calculate integarated allocation */
  Eigen::MatrixXd full_q_mat = wrench_matrix * integrated_rot;
  return full_q_mat;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(OrochiHxRobotModel, aerial_robot_model::RobotModel);
