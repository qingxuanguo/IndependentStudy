#ifndef THORMANG_PROJECT_MODULE_PROJECT_ONLINE_H_
#define THORMANG3_WALKING_MODULE_PROJECT_ONLINE_H_

#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "robotis_framework_common/singleton.h"
#include "robotis_math/robotis_math.h"

#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"
#include "thormang3_balance_control/thormang3_balance_control.h"

namespace thormang3
{

class ProjectOnline : public robotis_framework::Singleton<ProjectOnline>
{
public:
  ProjectOnline();
  virtual ~ProjectOnline();

  void initialize();
  void reInitialize();
  void start();
  void phase_1_start();
  void stop();
  void process();
  bool isRunning();

  void setCurrentIMUSensorOutput(double gyro_x, double gyro_y, double quat_x, double quat_y, double quat_z, double quat_w);

  double r_leg_out_angle_rad_[6];
  double l_leg_out_angle_rad_[6];
  double curr_angle_rad_[31]; /*2-13 legs 14-276 arms*/
  double out_angle_rad_[31];
  double init_joint_positions[31];

  Eigen::MatrixXd mat_cob_to_g_,  mat_g_to_cob_;
  Eigen::MatrixXd mat_robot_to_cob_, mat_cob_to_robot_;
  Eigen::MatrixXd mat_robot_to_g_, mat_g_to_robot_;
  Eigen::MatrixXd mat_cob_to_rhip_, mat_rhip_to_cob_;
  Eigen::MatrixXd mat_cob_to_lhip_, mat_lhip_to_cob_;

  Eigen::MatrixXd mat_g_to_rfoot_, mat_g_to_lfoot_;

  double current_right_fx_N_,  current_right_fy_N_,  current_right_fz_N_;
  double current_right_tx_Nm_, current_right_ty_Nm_, current_right_tz_Nm_;
  double current_left_fx_N_,  current_left_fy_N_,  current_left_fz_N_;
  double current_left_tx_Nm_, current_left_ty_Nm_, current_left_tz_Nm_;

  Eigen::Quaterniond quat_current_imu_;
  Eigen::MatrixXd mat_current_imu_;
  double current_imu_roll_rad_, current_imu_pitch_rad_;
  double current_gyro_roll_rad_per_sec_, current_gyro_pitch_rad_per_sec_;

  int balance_error_;
  thormang3::BalanceControlUsingPDController balance_ctrl_;

  double measuredZMP_r_x, measuredZMP_r_y, measuredZMP_l_x, measuredZMP_l_y;
  double measuredZMP_px, measuredZMP_py;

private:
  double wsigmoid(double time, double period, double time_shift, double mag, double mag_shift, double sigmoid_ratio, double distortion_ratio);

  KinematicsDynamics* thormang3_kd_;

  double total_mass_of_robot_;
  double right_dsp_fz_N_, right_ssp_fz_N_;
  double left_dsp_fz_N_,  left_ssp_fz_N_;

  Eigen::MatrixXd mat_robot_to_cob_modified_, mat_cob_to_robot_modified_;
  Eigen::MatrixXd mat_robot_to_rf_modified_;
  Eigen::MatrixXd mat_robot_to_lf_modified_;
  Eigen::MatrixXd mat_robot_to_rfoot_;
  Eigen::MatrixXd mat_robot_to_lfoot_;

  robotis_framework::Pose3D initial_right_foot_pose_, initial_left_foot_pose_, initial_body_pose_;
  robotis_framework::Pose3D present_right_foot_pose_, present_left_foot_pose_, present_body_pose_;
  robotis_framework::Pose3D previous_step_right_foot_pose_, previous_step_left_foot_pose_, previous_step_body_pose_;
  robotis_framework::Pose3D rhip_to_rfoot_pose_, lhip_to_lfoot_pose_;
  robotis_framework::FifthOrderPolynomialTrajectory foot_x_traj1, foot_x_traj2, foot_x_traj3, foot_y_traj, foot_z_traj1, foot_z_traj2, foot_z_traj3;
  // robotis_framework::FifthOrderPolynomialTrajectory foot_roll_tra_, foot_pitch_tra_, foot_yaw_tra_;
  // robotis_framework::FifthOrderPolynomialTrajectory foot_z_swap_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory com_z_traj, com_z_traj_end;
  robotis_framework::FifthOrderPolynomialTrajectory com_x_traj11, com_x_traj12, com_y_traj11, com_y_traj12;
  robotis_framework::FifthOrderPolynomialTrajectory com_x_traj2, com_y_traj2;
  robotis_framework::FifthOrderPolynomialTrajectory com_x_traj31, com_x_traj32, com_y_traj31, com_y_traj32;
  // robotis_framework::FifthOrderPolynomialTrajectory waist_yaw_tra_;
  // robotis_framework::FifthOrderPolynomialTrajectory body_z_swap_tra_;
  // robotis_framework::FifthOrderPolynomialTrajectory hip_roll_swap_tra_;

  Eigen::MatrixXd mat_rfoot_to_rft_, mat_lfoot_to_lft_;
  Eigen::MatrixXd rot_x_pi_3d_, rot_z_pi_3d_;

  boost::mutex imu_data_mutex_lock_;

  bool real_running, ctrl_running;
  bool is_walking_task;

  double current_time;    //Absolute Time
  double reference_time_;  //Absolute Time
  double control_cycle_index;
  double Tss, Tds, Tend, Tcshift, T_foot_1, T_foot_2, T_foot_3;

  int     mov_size_, mov_step_;
  double  mov_time_;
  bool init_balance_offset_;
  int  balancing_index_;

  void calcDesiredPose();
  void parseIniPoseData(const std::string &path);
};

}

#endif
