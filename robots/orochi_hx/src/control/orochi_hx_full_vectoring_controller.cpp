#include <orochi_hx/control/orochi_hx_full_vectoring_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

OrochiHxFullVectoringController::OrochiHxFullVectoringController()
  : PoseLinearController(), candidate_yaw_term_(0), torque_allocation_matrix_inv_pub_stamp_(0), prev_rotation_stamp_(0)
{
  curr_target_baselink_rot_.setRPY(0, 0, 0);
  final_target_baselink_rot_.setRPY(0, 0, 0);
}

void OrochiHxFullVectoringController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                                 boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                 boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                                 double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  orochi_hx_robot_model_ = boost::dynamic_pointer_cast<OrochiHxFullVectoringRobotModel>(robot_model_);
  if (!orochi_hx_robot_model_)
    throw std::runtime_error("[OrochiHxFullVectoringController] robot model is not OrochiHxFullVectoringRobotModel, "
                             "please set robot_model_plugin_name: orochi_hx_full_vectoring_robot_model");
  robot_model_for_control_ = boost::make_shared<OrochiHxFullVectoringRobotModel>();

  target_acc_cog_ = Eigen::VectorXd::Zero(6);
  target_vectoring_f_ = Eigen::VectorXd::Zero(3 * motor_num_);
  target_base_thrust_.resize(motor_num_, 0.0);
  target_gimbal_roll_.resize(motor_num_, 0.0);
  target_gimbal_pitch_.resize(motor_num_, 0.0);
  target_joint_roll_.resize(motor_num_, 0.0);
  q_mat_.resize(6, motor_num_);
  q_mat_inv_.resize(motor_num_, 6);

  rosParamInit();

  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  servo_torque_command_pub_ = nh_.advertise<spinal::ServoTorqueCmd>("servo/torque_enable", 1);
  torque_allocation_matrix_inv_pub_ =
      nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
  target_acc_cog_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_acc_cog", 1);
  desire_coord_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);

  joint_roll_ctrl_sub_ =
      nh_.subscribe("joint_roll_ctrl", 1, &OrochiHxFullVectoringController::jointRollCtrlCallback, this);
  final_target_baselink_rpy_sub_ = nh_.subscribe("final_target_baselink_rpy", 1,
                                                 &OrochiHxFullVectoringController::targetBaselinkRPYCallback, this);
}

void OrochiHxFullVectoringController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");

  getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_,
                   0.05);
  getParam<bool>(control_nh, "use_fc_for_att_control", use_fc_for_att_control_, true);
  getParam<double>(control_nh, "baselink_rot_change_thresh", baselink_rot_change_thresh_,
                   0.02);  // the max change of the baselink rotation in each update [rad]
  getParam<double>(control_nh, "baselink_rot_pub_interval", baselink_rot_pub_interval_,
                   0.1);  // the interval to update the baselink rotation [sec]

  /* initial target joint roll (phi_j) for each module */
  std::vector<double> init_joint_roll;
  if (control_nh.getParam("init_joint_roll", init_joint_roll))
  {
    if ((int)init_joint_roll.size() != motor_num_)
      ROS_ERROR("[OrochiHxFullVectoringController] the size of init_joint_roll (%d) is not equal to motor num (%d)",
                (int)init_joint_roll.size(), motor_num_);
    else
      target_joint_roll_ = init_joint_roll;
  }
}

void OrochiHxFullVectoringController::reset()
{
  PoseLinearController::reset();

  setAttitudeGains();

  torque_allocation_matrix_inv_pub_stamp_ = -1;

  /* reset the baselink rotation to level */
  tf::Quaternion level;
  level.setRPY(0, 0, 0);
  {
    std::lock_guard<std::mutex> lock(final_target_baselink_rot_mutex_);
    final_target_baselink_rot_ = level;
  }
  setCurrentTargetBaselinkRot(level);
}

void OrochiHxFullVectoringController::activate()
{
  ControlBase::activate();

  // enable all servos when activated
  spinal::ServoTorqueCmd servo_torque_command;
  for (int i = 0; i < robot_model_->getJointNum(); i++)
  {
    servo_torque_command.index.push_back(i);
    servo_torque_command.torque_enable.push_back(1);
  }
  servo_torque_command_pub_.publish(servo_torque_command);

  // set gimbal angles to zero (keep the joint roll)
  for (int i = 0; i < motor_num_; i++)
  {
    target_gimbal_roll_.at(i) = 0.0;
    target_gimbal_pitch_.at(i) = 0.0;
  }
  sendGimbalAngles();
}

bool OrochiHxFullVectoringController::update()
{
  if (!PoseLinearController::update())
    return false;

  return true;
}

void OrochiHxFullVectoringController::controlCore()
{
  baselinkRotationProcess();

  PoseLinearController::controlCore();

  /* update robot model for control */
  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_control_->setCogDesireOrientation(cog_desire_orientation);
  robot_model_for_control_->setExtraModuleMap(robot_model_->getExtraModuleMap());
  robot_model_for_control_->updateRobotModel(robot_model_->getJointPositions());

  /* calculate feedback term */
  calcAccFromCog();
  forceLandingProcess();

  /* calculate control input */
  fullVectoringAllocation();

  /* for spinal */
  processGimbalAngles();
  calcYawTerm();
}

void OrochiHxFullVectoringController::calcAccFromCog()
{
  tf::Quaternion cog2baselink_rot;
  tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
  tf::Matrix3x3 cog_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_) *
                          tf::Matrix3x3(cog2baselink_rot).inverse();  // w_R_b * cog_R_b.inverse() = w_R_cog

  tf::Vector3 target_linear_acc_w(pid_controllers_.at(X).result(), pid_controllers_.at(Y).result(),
                                  pid_controllers_.at(Z).result());
  tf::Vector3 target_linear_acc_cog = cog_rot.inverse() * target_linear_acc_w;

  Eigen::VectorXd target_acc_cog = Eigen::VectorXd::Zero(6);
  target_acc_cog.head(3) =
      Eigen::Vector3d(target_linear_acc_cog.x(), target_linear_acc_cog.y(), target_linear_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  Eigen::Vector3d omega;
  tf::vectorTFToEigen(omega_, omega);
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::Vector3d gyro = omega.cross(inertia * omega);

  target_acc_cog.tail(3) =
      Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z) + inertia.inverse() * gyro;

  target_acc_cog_ = target_acc_cog;
}

void OrochiHxFullVectoringController::forceLandingProcess()
{
  if (navigator_->getForceLandingFlag() &&
      pid_controllers_.at(Z).result() < 5.0)  // heuristic measures to avoid to large gimbal angles after force land
    start_rp_integration_ = false;
}

void OrochiHxFullVectoringController::fullVectoringAllocation()
{
  /* 1. allocation matrix: wrench = Q * [f_1; ...; f_N], f_i is the thrust force in the link (female{i}) frame */
  std::vector<Eigen::Matrix3d> links_rotation_from_cog =
      robot_model_for_control_->getLinksRotationFromCog<Eigen::Matrix3d>();
  std::vector<Eigen::Vector3d> rotors_origin_from_cog =
      robot_model_for_control_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const double m_f_rate = robot_model_for_control_->getMFRate();
  const std::map<int, int> rotor_direction = robot_model_for_control_->getRotorDirection();

  Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3 * motor_num_);
  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
  for (int i = 0; i < motor_num_; i++)
  {
    wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i)) +
                                   m_f_rate * rotor_direction.at(i + 1) * Eigen::Matrix3d::Identity();
    full_q_mat.middleCols(3 * i, 3) = wrench_map * links_rotation_from_cog.at(i);
  }

  /* 2. target wrench */
  Eigen::VectorXd target_wrench = Eigen::VectorXd::Zero(6);
  target_wrench.head(3) = robot_model_for_control_->getMass() * target_acc_cog_.head(3);
  target_wrench.tail(3) = robot_model_for_control_->getInertia<Eigen::Matrix3d>() * target_acc_cog_.tail(3);

  /* 3. vectoring force */
  target_vectoring_f_ = aerial_robot_model::pseudoinverse(full_q_mat) * target_wrench;

  /* 4. thrust and 2DoF gimbal angles: f_i = lambda_i * Rx(-phi_g) * Ry(th_g) * e_z */
  for (int i = 0; i < motor_num_; i++)
  {
    Eigen::Vector3d f_i = target_vectoring_f_.segment(3 * i, 3);
    target_base_thrust_.at(i) = std::clamp(f_i.norm(), robot_model_->getThrustLowerLimit(),
                                           robot_model_->getThrustUpperLimit());

    /* before takeoff (and during force landing), keep the gimbals neutral to avoid large motion on the ground */
    if (!start_rp_integration_)
    {
      target_gimbal_roll_.at(i) = 0.0;
      target_gimbal_pitch_.at(i) = 0.0;
      continue;
    }

    double gimbal_roll = atan2(f_i.y(), f_i.z());
    double gimbal_pitch = atan2(f_i.x(), f_i.y() * sin(gimbal_roll) + f_i.z() * cos(gimbal_roll));
    target_gimbal_roll_.at(i) = gimbal_roll;
    target_gimbal_pitch_.at(i) = gimbal_pitch;
  }
}

void OrochiHxFullVectoringController::processGimbalAngles()
{
  /* allocation matrix for the attitude control in spinal, based on the current rotor normals */
  q_mat_ = robot_model_for_control_->calcWrenchMatrixOnCoG();
  q_mat_inv_ = aerial_robot_model::pseudoinverse(q_mat_);
}

void OrochiHxFullVectoringController::calcYawTerm()
{
  // special process for yaw since the bandwidth between PC and spinal
  double max_yaw_scale = 0;  // for reconstruct yaw control term in spinal
  for (int i = 0; i < motor_num_; i++)
  {
    if (q_mat_inv_(i, YAW) > max_yaw_scale)
      max_yaw_scale = q_mat_inv_(i, YAW);
  }
  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;
}

void OrochiHxFullVectoringController::baselinkRotationProcess()
{
  tf::Quaternion final_target_baselink_rot;
  {
    std::lock_guard<std::mutex> lock(final_target_baselink_rot_mutex_);
    /* return to level during landing and force landing */
    if (navigator_->getNaviState() == aerial_robot_navigation::LAND_STATE || navigator_->getForceLandingFlag())
      final_target_baselink_rot_.setRPY(0, 0, 0);
    final_target_baselink_rot = final_target_baselink_rot_;
  }

  if (curr_target_baselink_rot_ == final_target_baselink_rot)
    return;

  if (ros::Time::now().toSec() - prev_rotation_stamp_ < baselink_rot_pub_interval_)
    return;

  /* gradually change the target rotation (same as DragonNavigator::baselinkRotationProcess) */
  tf::Quaternion delta_q = curr_target_baselink_rot_.inverse() * final_target_baselink_rot;
  double angle = delta_q.getAngle();
  if (angle > M_PI)
    angle -= 2 * M_PI;

  tf::Quaternion next_target_baselink_rot = final_target_baselink_rot;
  if (fabs(angle) > baselink_rot_change_thresh_)
    next_target_baselink_rot =
        curr_target_baselink_rot_ * tf::Quaternion(delta_q.getAxis(), fabs(angle) / angle * baselink_rot_change_thresh_);

  setCurrentTargetBaselinkRot(next_target_baselink_rot);
  prev_rotation_stamp_ = ros::Time::now().toSec();
}

void OrochiHxFullVectoringController::setCurrentTargetBaselinkRot(const tf::Quaternion& rot)
{
  curr_target_baselink_rot_ = rot;

  /* the CoG frame is kept level by the controller, thus the baselink is rotated by this orientation */
  KDL::Rotation kdl_rot;
  tf::quaternionTFToKDL(curr_target_baselink_rot_, kdl_rot);
  robot_model_->setCogDesireOrientation(kdl_rot);

  /* send to spinal to rotate the attitude control frame in the same way */
  if (desire_coord_pub_)
  {
    spinal::DesireCoord msg;
    double r, p, y;
    tf::Matrix3x3(curr_target_baselink_rot_).getRPY(r, p, y);
    msg.roll = r;
    msg.pitch = p;
    msg.yaw = y;
    desire_coord_pub_.publish(msg);
  }
}

void OrochiHxFullVectoringController::targetBaselinkRPYCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  /* only roll and pitch: the yaw is controlled by the navigation */
  std::lock_guard<std::mutex> lock(final_target_baselink_rot_mutex_);
  final_target_baselink_rot_.setRPY(msg->vector.x, msg->vector.y, 0);
}

void OrochiHxFullVectoringController::sendCmd()
{
  PoseLinearController::sendCmd();

  std_msgs::Float32MultiArray target_vectoring_force_msg;
  for (int i = 0; i < target_vectoring_f_.size(); i++)
    target_vectoring_force_msg.data.push_back(target_vectoring_f_(i));
  target_vectoring_force_pub_.publish(target_vectoring_force_msg);

  std_msgs::Float32MultiArray target_acc_cog_msg;
  for (int i = 0; i < target_acc_cog_.size(); i++)
    target_acc_cog_msg.data.push_back(target_acc_cog_(i));
  target_acc_cog_pub_.publish(target_acc_cog_msg);

  sendGimbalAngles();

  sendFourAxisCommand();

  setAttitudeGains();

  sendTorqueAllocationMatrixInv();
}

void OrochiHxFullVectoringController::sendGimbalAngles()
{
  std::vector<double> target_joint_roll;
  {
    std::lock_guard<std::mutex> lock(target_joint_roll_mutex_);
    target_joint_roll = target_joint_roll_;
  }

  /* distribute the 2DoF gimbal and the joint roll to the servos (see manual_for_full_vectoring.md) */
  sensor_msgs::JointState gimbal_control_msg;
  gimbal_control_msg.header.stamp = ros::Time::now();
  for (int i = 0; i < motor_num_; i++)
  {
    std::string s = std::to_string(i + 1);
    gimbal_control_msg.name.push_back(std::string("gimbal_pitch") + s);  // th^g = th_g
    gimbal_control_msg.position.push_back(target_gimbal_pitch_.at(i));
    gimbal_control_msg.name.push_back(std::string("female_roll") + s);  // phi^f = phi_g
    gimbal_control_msg.position.push_back(target_gimbal_roll_.at(i));
    gimbal_control_msg.name.push_back(std::string("male_roll") + s);  // phi^m = phi_j - phi_g
    gimbal_control_msg.position.push_back(target_joint_roll.at(i) - target_gimbal_roll_.at(i));
  }
  gimbal_control_pub_.publish(gimbal_control_msg);
}

void OrochiHxFullVectoringController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;
  if (use_fc_for_att_control_)
    flight_command_data.angles[2] = candidate_yaw_term_;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_.publish(flight_command_data);
}

void OrochiHxFullVectoringController::sendTorqueAllocationMatrixInv()
{
  if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
  {
    torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now().toSec();

    spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
    Eigen::MatrixXd torque_allocation_matrix_inv = q_mat_inv_.rightCols(3);
    torque_allocation_matrix_inv_msg.rows.resize(motor_num_);

    if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
      ROS_ERROR("Torque Allocation Matrix overflow");
    for (int i = 0; i < motor_num_; i++)
    {
      torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i, 0) * 1000;
      torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i, 1) * 1000;
      torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i, 2) * 1000;
    }
    torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
  }
}

void OrochiHxFullVectoringController::setAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg;  // for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  rpy_gain_msg.motors.resize(1);
  if (use_fc_for_att_control_)
  {
    rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
    rpy_gain_msg.motors.at(0).roll_i = pid_controllers_.at(ROLL).getIGain() * 1000;
    rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
    rpy_gain_msg.motors.at(0).yaw_d = pid_controllers_.at(YAW).getDGain() * 1000;
  }
  rpy_gain_pub_.publish(rpy_gain_msg);
}

void OrochiHxFullVectoringController::jointRollCtrlCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(target_joint_roll_mutex_);

  if (msg->name.size() == 0)
  {
    /* no name: the positions are in the order of joint_roll1 ... joint_rollN */
    if ((int)msg->position.size() != motor_num_)
    {
      ROS_ERROR("[OrochiHxFullVectoringController] joint_roll_ctrl: the size of position (%d) is not equal to motor "
                "num (%d)",
                (int)msg->position.size(), motor_num_);
      return;
    }
    for (int i = 0; i < motor_num_; i++)
      target_joint_roll_.at(i) = msg->position.at(i);
    return;
  }

  if (msg->name.size() != msg->position.size())
  {
    ROS_ERROR("[OrochiHxFullVectoringController] joint_roll_ctrl: the size of name and position are different");
    return;
  }

  for (size_t k = 0; k < msg->name.size(); k++)
  {
    const std::string prefix("joint_roll");
    const std::string& name = msg->name.at(k);
    int index = -1;
    if (name.compare(0, prefix.size(), prefix) == 0)
    {
      try
      {
        index = std::stoi(name.substr(prefix.size())) - 1;
      }
      catch (const std::exception&)
      {
        index = -1;
      }
    }
    if (index < 0 || index >= motor_num_)
    {
      ROS_ERROR("[OrochiHxFullVectoringController] joint_roll_ctrl: invalid joint name %s", name.c_str());
      continue;
    }
    target_joint_roll_.at(index) = msg->position.at(k);
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::OrochiHxFullVectoringController, aerial_robot_control::ControlBase);
