// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <orochi_hx/model/orochi_hx_full_vectoring_robot_model.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <spinal/DesireCoord.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/ServoTorqueCmd.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>

namespace aerial_robot_control
{
/*
  Full vectoring controller of orochi_hx (see manual_for_full_vectoring.md)

  1. the target wrench is allocated to the 3D thrust force f_i of each rotor, expressed in the link (female{i}) frame
  2. f_i is converted to the thrust lambda_i and the 2DoF gimbal angles (phi_g, th_g), where
       f_i = lambda_i * Rx(-phi_g) * Ry(th_g) * e_z
  3. the gimbal angles and the target joint roll phi_j are distributed to the servos:
       gimbal_pitch{i} = th_g, female_roll{i} = phi_g, male_roll{i} = phi_j - phi_g
     (joint_yaw{i} = psi_j is commanded via "joints_ctrl" as usual)
  4. the target roll and pitch of the baselink (fc) are received from "final_target_baselink_rpy" (e.g. rotateCog() in
     robot_interface.py), and the CoG desire orientation is gradually changed in the same way as DragonNavigator
*/
class OrochiHxFullVectoringController : public PoseLinearController
{
public:
  OrochiHxFullVectoringController();
  ~OrochiHxFullVectoringController() = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_rate) override;

private:
  ros::Publisher rpy_gain_pub_;                      // for spinal
  ros::Publisher flight_cmd_pub_;                    // for spinal
  ros::Publisher gimbal_control_pub_;                // for servo bridge
  ros::Publisher servo_torque_command_pub_;          // for spinal
  ros::Publisher torque_allocation_matrix_inv_pub_;  // for spinal
  ros::Publisher target_vectoring_force_pub_;        // for debug
  ros::Publisher target_acc_cog_pub_;                // for debug
  ros::Publisher desire_coord_pub_;                  // for spinal
  ros::Subscriber joint_roll_ctrl_sub_;
  ros::Subscriber final_target_baselink_rpy_sub_;

  boost::shared_ptr<OrochiHxFullVectoringRobotModel> orochi_hx_robot_model_;
  boost::shared_ptr<OrochiHxFullVectoringRobotModel> robot_model_for_control_;

  bool use_fc_for_att_control_;
  Eigen::VectorXd target_acc_cog_;
  Eigen::VectorXd target_vectoring_f_;
  std::vector<float> target_base_thrust_;
  std::vector<double> target_gimbal_roll_;   // phi_g
  std::vector<double> target_gimbal_pitch_;  // th_g
  std::vector<double> target_joint_roll_;    // phi_j
  std::mutex target_joint_roll_mutex_;
  Eigen::MatrixXd q_mat_;
  Eigen::MatrixXd q_mat_inv_;
  double candidate_yaw_term_;
  double torque_allocation_matrix_inv_pub_stamp_;
  double torque_allocation_matrix_inv_pub_interval_;

  /* baselink rotation */
  tf::Quaternion curr_target_baselink_rot_;
  tf::Quaternion final_target_baselink_rot_;
  std::mutex final_target_baselink_rot_mutex_;
  double baselink_rot_change_thresh_;
  double baselink_rot_pub_interval_;
  double prev_rotation_stamp_;

  bool update() override;
  void reset() override;
  void controlCore() override;
  void activate() override;
  void rosParamInit();

  void calcAccFromCog();
  void forceLandingProcess();
  void fullVectoringAllocation();
  void processGimbalAngles();
  void calcYawTerm();
  void baselinkRotationProcess();
  void setCurrentTargetBaselinkRot(const tf::Quaternion& rot);

  void sendCmd() override;
  void sendGimbalAngles();
  void sendFourAxisCommand();
  void sendTorqueAllocationMatrixInv();
  void setAttitudeGains();

  void jointRollCtrlCallback(const sensor_msgs::JointStateConstPtr& msg);
  void targetBaselinkRPYCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
};
};  // namespace aerial_robot_control
