#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include "thormang_project_module/thormang_project_module.h"


using namespace thormang3;

static const double MMtoM = 0.001;
static const double TIME_UNIT = 0.008;

const int PROJECT_MAX_JOINT_ID = 30; // STARTING FROM 0

static const int BalancingPhase1 = 1;
static const int BalancingPhase2 = 2;
static const int BalancingPhase3 = 3;
static const int BalancingPhase4 = 4;
static const int BalancingPhase5 = 5;
static const int BalancingPhase6 = 6;
static const int BalancingPhase7 = 7;

bool is_balance = true; //for test and data recording

double dsp_ratio = 0.2;
double com_shift_ratio = 0.4;
double T_task = 28;
double stair_height = 91.6 * MMtoM; //m
double step_length = 250 * MMtoM; //m
double foot_x_boundary = 104 * MMtoM; //m
double foot_y_boundary = 108 * MMtoM; //m

double Twait = 2;
double Tlift = 2;
int iswait = 1;


ProjectOnline::ProjectOnline()
{
  thormang3_kd_ = new KinematicsDynamics(WholeBody);
  present_right_foot_pose_.x = 0.0;    present_right_foot_pose_.y = -0.5*thormang3_kd_->leg_side_offset_m_;
  present_right_foot_pose_.z = -0.630;
  present_right_foot_pose_.roll = 0.0; present_right_foot_pose_.pitch = 0.0; present_right_foot_pose_.yaw = 0.0;

  present_left_foot_pose_.x = 0.0;    present_left_foot_pose_.y = 0.5*thormang3_kd_->leg_side_offset_m_;
  present_left_foot_pose_.z = -0.630;
  present_left_foot_pose_.roll = 0.0; present_left_foot_pose_.pitch = 0.0; present_left_foot_pose_.yaw = 0.0;

  present_body_pose_.x = 0.0;    present_body_pose_.y = 0.0;     present_body_pose_.z = 0.0;
  present_body_pose_.roll = 0.0; present_body_pose_.pitch = 0.0; present_body_pose_.yaw = 0;

  previous_step_right_foot_pose_  = present_right_foot_pose_;
  previous_step_left_foot_pose_   = present_left_foot_pose_;
  previous_step_body_pose_        = present_body_pose_;

  initial_right_foot_pose_ = previous_step_right_foot_pose_;
  initial_left_foot_pose_  = previous_step_left_foot_pose_;
  initial_body_pose_       = previous_step_body_pose_;

  mat_cob_to_rhip_ = robotis_framework::getTranslation4D(0.0,       thormang3_kd_->thormang3_link_data_[ID_R_LEG_START]->relative_position_.coeff(1, 0), 0.0);
  mat_rhip_to_cob_ = robotis_framework::getTranslation4D(0.0, -1.0*(thormang3_kd_->thormang3_link_data_[ID_R_LEG_START]->relative_position_.coeff(1, 0)), 0.0);
  mat_cob_to_lhip_ = robotis_framework::getTranslation4D(0.0,       thormang3_kd_->thormang3_link_data_[ID_L_LEG_START]->relative_position_.coeff(1, 0), 0.0);
  mat_lhip_to_cob_ = robotis_framework::getTranslation4D(0.0, -1.0*(thormang3_kd_->thormang3_link_data_[ID_L_LEG_START]->relative_position_.coeff(1, 0)), 0.0);

  mat_rfoot_to_rft_ = robotis_framework::getRotation4d(M_PI,0,0);
  mat_lfoot_to_lft_ = robotis_framework::getRotation4d(M_PI,0,0);
  rot_x_pi_3d_ = robotis_framework::getRotationX(M_PI);
  rot_z_pi_3d_ = robotis_framework::getRotationZ(M_PI);


  mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(present_body_pose_.x, present_body_pose_.y, present_body_pose_.z,
      present_body_pose_.roll, present_body_pose_.pitch, present_body_pose_.yaw);

  mat_cob_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_cob_);

  mat_robot_to_cob_ = robotis_framework::getRotation4d(present_body_pose_.roll, present_body_pose_.pitch, 0);
  mat_cob_to_robot_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_);
  mat_g_to_robot_   = mat_g_to_cob_ * mat_cob_to_robot_;
  mat_robot_to_g_   = robotis_framework::getInverseTransformation(mat_g_to_robot_);

  // balancing
  current_right_fx_N_  = current_right_fy_N_  = current_right_fz_N_  = 0;
  current_right_tx_Nm_ = current_right_ty_Nm_ = current_right_tz_Nm_ = 0;
  current_left_fx_N_  = current_left_fy_N_  = current_left_fz_N_  = 0;
  current_left_tx_Nm_ = current_left_ty_Nm_ = current_left_tz_Nm_ = 0;

  current_imu_roll_rad_ = current_imu_pitch_rad_ = 0;
  current_gyro_roll_rad_per_sec_ = current_gyro_pitch_rad_per_sec_ = 0;

  total_mass_of_robot_ = thormang3_kd_->calcTotalMass(0);

  right_dsp_fz_N_ = -1.0*(total_mass_of_robot_)*9.8*0.5;
  right_ssp_fz_N_ = -1.0*(total_mass_of_robot_)*9.8;
  left_dsp_fz_N_  = -1.0*(total_mass_of_robot_)*9.8*0.5;
  left_ssp_fz_N_  = -1.0*(total_mass_of_robot_)*9.8;

  balance_error_ = thormang3::BalanceControlError::NoError;
  quat_current_imu_.w() = cos(0.5*M_PI);
  quat_current_imu_.x() = sin(0.5*M_PI);
  quat_current_imu_.y() = 0;
  quat_current_imu_.z() = 0;

  real_running    = false;
  ctrl_running    = false;
  is_walking_task = 0;
}

ProjectOnline::~ProjectOnline()
{  }

void ProjectOnline::setCurrentIMUSensorOutput(double gyro_x, double gyro_y, double quat_x, double quat_y, double quat_z, double quat_w)
{
  imu_data_mutex_lock_.lock();

  current_gyro_roll_rad_per_sec_  = gyro_x;
  current_gyro_pitch_rad_per_sec_ = gyro_y;

  quat_current_imu_ = Eigen::Quaterniond(quat_w, quat_x, quat_y, quat_z);

  mat_current_imu_ = (rot_x_pi_3d_ * quat_current_imu_.toRotationMatrix()) * rot_z_pi_3d_;

  current_imu_roll_rad_  = atan2( mat_current_imu_.coeff(2,1), mat_current_imu_.coeff(2,2));
  current_imu_pitch_rad_ = atan2(-mat_current_imu_.coeff(2,0), sqrt(robotis_framework::powDI(mat_current_imu_.coeff(2,1), 2) + robotis_framework::powDI(mat_current_imu_.coeff(2,2), 2)));

  imu_data_mutex_lock_.unlock();
}

void ProjectOnline::initialize()
{
  if(real_running)
    return;

  balance_ctrl_.initialize(TIME_UNIT*1000.0);
  balance_ctrl_.setGyroBalanceEnable(true);
  balance_ctrl_.setOrientationBalanceEnable(true);
  balance_ctrl_.setForceTorqueBalanceEnable(true);

  init_balance_offset_ = false;

  std::string ini_pose_path = ros::package::getPath("thormang_project_module") + "/data/ini_pose.yaml";
  parseIniPoseData(ini_pose_path);

  current_time = 0; reference_time_ = 0; control_cycle_index = 0;
  Tss = T_task*(1-dsp_ratio)/2; // start of single support
  Tcshift = Tss*com_shift_ratio;
  Tds = Tss+T_task*dsp_ratio; // end of double support
  Tend = T_task;

  //phase 1 traj [mm]
  com_z_traj.changeTrajectory(0.0, 0.0, 0.0, 0.0, Tend, 0, 0.0, 0.0);
  com_x_traj11.changeTrajectory(0.0, 0.0, 0.0, 0.0, Tcshift, 0.0, 0.0, 0.0);
  com_y_traj11.changeTrajectory(0.0, 0.0, 0.0, 0.0, Tcshift, foot_y_boundary + 0.015, 0.0, 0.0);
  com_x_traj12.changeTrajectory(Tcshift, 0.0, 0.0, 0.0, Tss, 0, 0.0, 0.0);
  com_y_traj12.changeTrajectory(Tcshift, foot_y_boundary + 0.015, 0.0, 0.0, Tss, foot_y_boundary, 0.0, 0.0);

  com_x_traj31.changeTrajectory(Tds, step_length, 0.0, 0.0, Tend-Tcshift, step_length, 0.0, 0.0);
  com_y_traj31.changeTrajectory(Tds, -foot_y_boundary, 0.0, 0.0, Tend-Tcshift, -foot_y_boundary - 0.02, 0.0, 0.0);
  com_x_traj32.changeTrajectory(Tend-Tcshift, step_length, 0.0, 0.0, Tend, step_length, 0.0, 0.0);
  com_y_traj32.changeTrajectory(Tend-Tcshift, -foot_y_boundary - 0.02, 0.0, 0.0, Tend, 0.0, 0.0, 0.0);

  com_x_traj2.changeTrajectory(Tss, com_x_traj12.getPosition(Tss), com_x_traj12.getVelocity(Tss), com_x_traj12.getAcceleration(Tss),
                                Tds, com_x_traj31.getPosition(Tds), com_x_traj31.getVelocity(Tds), com_x_traj31.getAcceleration(Tds));
  com_y_traj2.changeTrajectory(Tss, com_y_traj12.getPosition(Tss), com_y_traj12.getVelocity(Tss), com_y_traj12.getAcceleration(Tss),
                                Tds, com_y_traj31.getPosition(Tds), com_y_traj31.getVelocity(Tds), com_y_traj31.getAcceleration(Tds));

  com_z_traj_end.changeTrajectory(0.0, 0.0, 0.0, 0.0, Tlift, stair_height+0.0, 0.0, 0.0);// lifting body absolute time

  T_foot_1 = 0.3*(Tss-Tcshift); //universal time stamp
  T_foot_2 = (0.4 + 0.3)*(Tss-Tcshift);
  foot_x_traj1.changeTrajectory(0.0, 0.0, 0.0, 0.0, T_foot_1, 0, 0.0, 0.0);
  foot_x_traj2.changeTrajectory(T_foot_1, foot_x_traj1.getPosition(T_foot_1), foot_x_traj1.getVelocity(T_foot_1), foot_x_traj1.getAcceleration(T_foot_1),
                                 T_foot_2, step_length, 0.0, 0.0);
  foot_x_traj3.changeTrajectory(T_foot_2, foot_x_traj2.getPosition(T_foot_2), foot_x_traj2.getVelocity(T_foot_2), foot_x_traj2.getAcceleration(T_foot_2),
                                 Tss-Tcshift, step_length, 0.0, 0.0);
  foot_y_traj.changeTrajectory(0.0, 0.0, 0.0, 0.0, Tss-Tcshift, 0.0, 0.0, 0.0);
  foot_z_traj1.changeTrajectory(0.0, 0.0, 0.0, 0.0, T_foot_1, 0.05 + stair_height, 0.0, 0.0);
  foot_z_traj2.changeTrajectory(T_foot_1, foot_z_traj1.getPosition(T_foot_1), foot_z_traj1.getVelocity(T_foot_1), foot_z_traj1.getAcceleration(T_foot_1),
                                 T_foot_2, foot_z_traj1.getPosition(T_foot_1), 0.0, 0.0);
  foot_z_traj3.changeTrajectory(T_foot_2, foot_z_traj2.getPosition(T_foot_2), foot_z_traj2.getVelocity(T_foot_2), foot_z_traj2.getAcceleration(T_foot_2),
                                 Tss-Tcshift, stair_height, 0.0, 0.0);

  present_right_foot_pose_ = previous_step_right_foot_pose_;
  present_left_foot_pose_  = previous_step_left_foot_pose_;
  present_body_pose_       = previous_step_body_pose_;

  ctrl_running = false;


  for(int angle_idx = 0; angle_idx < PROJECT_MAX_JOINT_ID+1; angle_idx++)
  {
    out_angle_rad_[angle_idx] = init_joint_positions[angle_idx];
  }

  measuredZMP_r_x = 0; measuredZMP_r_y = 0; measuredZMP_l_x = 0; measuredZMP_l_y = 0;
  measuredZMP_px = 0; measuredZMP_py = 0;
}

void ProjectOnline::reInitialize()
{
  if(real_running)
    return;

}

void ProjectOnline::start()
{
  ctrl_running = true;
  real_running = true;
}

void ProjectOnline::phase_1_start()
{
  ctrl_running = true;
  real_running = true;
  is_walking_task = 1;
}

void ProjectOnline::stop()
{
  ctrl_running = false;
}

void ProjectOnline::calcDesiredPose()
{
  present_body_pose_.x = 0;
  present_body_pose_.y = 0;
}

void ProjectOnline::process()
{
  if(!ctrl_running)
  {
    return;
  }
  else
  {
    current_time = control_cycle_index*TIME_UNIT;//current time

    double curr_x_com, curr_y_com;
    double curr_xdd_com, curr_ydd_com;

    if (is_walking_task == 1)
    {
      if(current_time > Tend + Tlift)
      {
        stop();
        control_cycle_index = 0;
        is_walking_task = 0;
      }
      if (current_time >= (Tds + Twait) && iswait == 1)
      {
        iswait = 0;
        current_time = current_time - Twait;
        control_cycle_index = control_cycle_index - Twait/TIME_UNIT;
      }
      // step_data_mutex_lock_.lock();

      calcDesiredPose();

      measuredZMP_r_x = (current_right_ty_Nm_-current_right_fx_N_*(present_right_foot_pose_.z-initial_right_foot_pose_.z + 0.02))/current_right_fz_N_;
      measuredZMP_r_y = (current_right_tx_Nm_-current_right_fy_N_*(present_right_foot_pose_.z-initial_right_foot_pose_.z + 0.02))/current_right_fz_N_;
      measuredZMP_l_x = (current_left_ty_Nm_-current_left_fx_N_*(present_left_foot_pose_.z-initial_left_foot_pose_.z + 0.02))/current_left_fz_N_;
      measuredZMP_l_y = (current_left_tx_Nm_-current_left_fy_N_*(present_left_foot_pose_.z-initial_left_foot_pose_.z + 0.02))/current_left_fz_N_;

      // for(int angle_idx = 0; angle_idx < 6; angle_idx++)/*leg*/
      // {
      //   out_angle_rad_[angle_idx+0] = curr_angle_rad_[angle_idx+0]+0.05;/*right leg*/
      //   out_angle_rad_[angle_idx+6] = curr_angle_rad_[angle_idx+6]+0.05;/*left leg*/
      // }
      // for(int angle_idx = 12; angle_idx < 19; angle_idx++)/*arm*/
      // {
      //   out_angle_rad_[angle_idx+0] = curr_angle_rad_[angle_idx+0]+0.05;/*right arm*/
      //   out_angle_rad_[angle_idx+7] = curr_angle_rad_[angle_idx+7]+0.05;/*left arm*/
      // }
      // out_angle_rad_[26] = curr_angle_rad_[26]+0.05;/*torso*/
      // for(int angle_idx = 27; angle_idx < PROJECT_MAX_JOINT_ID+1; angle_idx++)/*gripper head*/
      // {
      //   out_angle_rad_[angle_idx] = curr_angle_rad_[angle_idx];
      // }

      if (current_time>=0 && current_time<=Tcshift)
      {
        mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(com_x_traj11.getPosition(current_time),
                                                                  -com_y_traj11.getPosition(current_time),
                                                                  com_z_traj.getPosition(current_time), 0.0, 0.0, 0.0);

        mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(present_right_foot_pose_.x, present_right_foot_pose_.y, present_right_foot_pose_.z,
                                                                      present_right_foot_pose_.roll, present_right_foot_pose_.pitch, present_right_foot_pose_.yaw);

        mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(present_left_foot_pose_.x, present_left_foot_pose_.y, present_left_foot_pose_.z,
                                                                      present_left_foot_pose_.roll, present_left_foot_pose_.pitch, present_left_foot_pose_.yaw);
        curr_x_com = com_x_traj11.getPosition(current_time);
        curr_y_com = com_y_traj11.getPosition(current_time);
        curr_xdd_com = com_x_traj11.getAcceleration(current_time);
        curr_ydd_com = com_y_traj11.getAcceleration(current_time);

        measuredZMP_px = (measuredZMP_r_x * current_right_fz_N_ + measuredZMP_l_x * current_left_fz_N_) / (current_right_fz_N_ + current_left_fz_N_);
        measuredZMP_py = (measuredZMP_r_y * current_right_fz_N_ + measuredZMP_l_y * current_left_fz_N_) / (current_right_fz_N_ + current_left_fz_N_);

        balancing_index_ = BalancingPhase1;
      }
      else if (current_time > Tcshift && current_time <= Tss)
      {
        if (current_time <= Tcshift + T_foot_1)
        {
          present_left_foot_pose_.x = initial_left_foot_pose_.x + foot_x_traj1.getPosition(current_time-Tcshift);
          present_left_foot_pose_.y = initial_left_foot_pose_.y + foot_y_traj.getPosition(current_time-Tcshift);
          present_left_foot_pose_.z = initial_left_foot_pose_.z + foot_z_traj1.getPosition(current_time-Tcshift);
        }
        else if (current_time <= Tcshift + T_foot_2)
        {
          present_left_foot_pose_.x = initial_left_foot_pose_.x + foot_x_traj2.getPosition(current_time-Tcshift);
          present_left_foot_pose_.y = initial_left_foot_pose_.y + foot_y_traj.getPosition(current_time-Tcshift);
          present_left_foot_pose_.z = initial_left_foot_pose_.z + foot_z_traj2.getPosition(current_time-Tcshift);
        }
        else
        {
          present_left_foot_pose_.x = initial_left_foot_pose_.x + foot_x_traj3.getPosition(current_time-Tcshift);
          present_left_foot_pose_.y = initial_left_foot_pose_.y + foot_y_traj.getPosition(current_time-Tcshift);
          present_left_foot_pose_.z = initial_left_foot_pose_.z + foot_z_traj3.getPosition(current_time-Tcshift);
        }

        mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(com_x_traj12.getPosition(current_time),
                                                                  -com_y_traj12.getPosition(current_time),
                                                                  com_z_traj.getPosition(current_time), 0.0, 0.0, 0.0);

        mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(present_right_foot_pose_.x, present_right_foot_pose_.y, present_right_foot_pose_.z,
                                                                      present_right_foot_pose_.roll, present_right_foot_pose_.pitch, present_right_foot_pose_.yaw);

        mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(present_left_foot_pose_.x, present_left_foot_pose_.y, present_left_foot_pose_.z,
                                                                      present_left_foot_pose_.roll, present_left_foot_pose_.pitch, present_left_foot_pose_.yaw);
        curr_x_com = com_x_traj12.getPosition(current_time);
        curr_y_com = com_y_traj12.getPosition(current_time);
        curr_xdd_com = com_x_traj12.getAcceleration(current_time);
        curr_ydd_com = com_y_traj12.getAcceleration(current_time);

        measuredZMP_px = measuredZMP_l_x;
        measuredZMP_py = measuredZMP_l_y;

        balancing_index_ = BalancingPhase2;
      }
      else if (current_time>Tss && current_time<=Tds)
      {
        present_left_foot_pose_.x = initial_left_foot_pose_.x + foot_x_traj3.getPosition(Tss);
        present_left_foot_pose_.y = initial_left_foot_pose_.y + foot_y_traj.getPosition(Tss);
        present_left_foot_pose_.z = initial_left_foot_pose_.z + foot_z_traj3.getPosition(Tss);

        mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(com_x_traj2.getPosition(current_time),
                                                                  -com_y_traj2.getPosition(current_time),
                                                                  com_z_traj.getPosition(current_time), 0.0, 0.0, 0.0);

        mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(present_right_foot_pose_.x, present_right_foot_pose_.y, present_right_foot_pose_.z,
                                                                      present_right_foot_pose_.roll, present_right_foot_pose_.pitch, present_right_foot_pose_.yaw);

        mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(present_left_foot_pose_.x, present_left_foot_pose_.y, present_left_foot_pose_.z,
                                                                      present_left_foot_pose_.roll, present_left_foot_pose_.pitch, present_left_foot_pose_.yaw);
        curr_x_com = com_x_traj2.getPosition(current_time);
        curr_y_com = com_y_traj2.getPosition(current_time);
        curr_xdd_com = com_x_traj2.getAcceleration(current_time);
        curr_ydd_com = com_y_traj2.getAcceleration(current_time);

        measuredZMP_px = (measuredZMP_r_x * current_right_fz_N_ + measuredZMP_l_x * current_left_fz_N_) / (current_right_fz_N_ + current_left_fz_N_);
        measuredZMP_py = (measuredZMP_r_y * current_right_fz_N_ + measuredZMP_l_y * current_left_fz_N_) / (current_right_fz_N_ + current_left_fz_N_);

        balancing_index_ = BalancingPhase3;
      }
      else if (current_time>Tds && current_time<=(Tds + Twait) && iswait == 1)//double support wait
      {
        present_left_foot_pose_.x = initial_left_foot_pose_.x + foot_x_traj3.getPosition(Tss);
        present_left_foot_pose_.y = initial_left_foot_pose_.y + foot_y_traj.getPosition(Tss);
        present_left_foot_pose_.z = initial_left_foot_pose_.z + foot_z_traj3.getPosition(Tss);

        present_right_foot_pose_.x = initial_right_foot_pose_.x;
        present_right_foot_pose_.y = initial_right_foot_pose_.y;
        present_right_foot_pose_.z = initial_right_foot_pose_.z;

        mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(com_x_traj2.getPosition(Tds),
                                                                  -com_y_traj2.getPosition(Tds),
                                                                  com_z_traj.getPosition(Tds), 0.0, 0.0, 0.0);

        mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(present_right_foot_pose_.x, present_right_foot_pose_.y, present_right_foot_pose_.z,
                                                                      present_right_foot_pose_.roll, present_right_foot_pose_.pitch, present_right_foot_pose_.yaw);

        mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(present_left_foot_pose_.x, present_left_foot_pose_.y, present_left_foot_pose_.z,
                                                                      present_left_foot_pose_.roll, present_left_foot_pose_.pitch, present_left_foot_pose_.yaw);
        curr_x_com = com_x_traj2.getPosition(current_time);
        curr_y_com = com_y_traj2.getPosition(current_time);
        curr_xdd_com = com_x_traj2.getAcceleration(current_time);
        curr_ydd_com = com_y_traj2.getAcceleration(current_time);

        measuredZMP_px = (measuredZMP_r_x * current_right_fz_N_ + measuredZMP_l_x * current_left_fz_N_) / (current_right_fz_N_ + current_left_fz_N_);
        measuredZMP_py = (measuredZMP_r_y * current_right_fz_N_ + measuredZMP_l_y * current_left_fz_N_) / (current_right_fz_N_ + current_left_fz_N_);

        balancing_index_ = BalancingPhase4;
      }
      else if (current_time > Tds && current_time <= (Tend - Tcshift) && iswait == 0)
      {
        present_left_foot_pose_.x = initial_left_foot_pose_.x + foot_x_traj3.getPosition(Tss);
        present_left_foot_pose_.y = initial_left_foot_pose_.y + foot_y_traj.getPosition(Tss);
        present_left_foot_pose_.z = initial_left_foot_pose_.z + foot_z_traj3.getPosition(Tss);

        if (current_time <= Tds + T_foot_1)
        {
          present_right_foot_pose_.x = initial_right_foot_pose_.x + foot_x_traj1.getPosition(current_time-Tds);
          present_right_foot_pose_.y = initial_right_foot_pose_.y + foot_y_traj.getPosition(current_time-Tds);
          present_right_foot_pose_.z = initial_right_foot_pose_.z + foot_z_traj1.getPosition(current_time-Tds);
        }
        else if (current_time <= Tds + T_foot_2)
        {
          present_right_foot_pose_.x = initial_right_foot_pose_.x + foot_x_traj2.getPosition(current_time-Tds);
          present_right_foot_pose_.y = initial_right_foot_pose_.y + foot_y_traj.getPosition(current_time-Tds);
          present_right_foot_pose_.z = initial_right_foot_pose_.z + foot_z_traj2.getPosition(current_time-Tds);
        }
        else
        {
          present_right_foot_pose_.x = initial_right_foot_pose_.x + foot_x_traj3.getPosition(current_time-Tds);
          present_right_foot_pose_.y = initial_right_foot_pose_.y + foot_y_traj.getPosition(current_time-Tds);
          present_right_foot_pose_.z = initial_right_foot_pose_.z + foot_z_traj3.getPosition(current_time-Tds);
        }

        mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(com_x_traj31.getPosition(current_time),
                                                                  -com_y_traj31.getPosition(current_time),
                                                                  com_z_traj.getPosition(current_time), 0.0, 0.0, 0.0);

        mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(present_right_foot_pose_.x, present_right_foot_pose_.y, present_right_foot_pose_.z,
                                                                      present_right_foot_pose_.roll, present_right_foot_pose_.pitch, present_right_foot_pose_.yaw);

        mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(present_left_foot_pose_.x, present_left_foot_pose_.y, present_left_foot_pose_.z,
                                                                      present_left_foot_pose_.roll, present_left_foot_pose_.pitch, present_left_foot_pose_.yaw);
        curr_x_com = com_x_traj31.getPosition(current_time);
        curr_y_com = com_y_traj31.getPosition(current_time);
        curr_xdd_com = com_x_traj31.getAcceleration(current_time);
        curr_ydd_com = com_y_traj31.getAcceleration(current_time);

        measuredZMP_px = measuredZMP_r_x;
        measuredZMP_py = measuredZMP_r_y;

        balancing_index_ = BalancingPhase5;
      }
      else if (current_time>(Tend - Tcshift) && current_time<=Tend && iswait == 0)
      {
        present_right_foot_pose_.x = initial_right_foot_pose_.x + foot_x_traj3.getPosition(Tss);
        present_right_foot_pose_.y = initial_right_foot_pose_.y + foot_y_traj.getPosition(Tss);
        present_right_foot_pose_.z = initial_right_foot_pose_.z + foot_z_traj3.getPosition(Tss);

        present_left_foot_pose_.x = initial_left_foot_pose_.x + foot_x_traj3.getPosition(Tss);
        present_left_foot_pose_.y = initial_left_foot_pose_.y + foot_y_traj.getPosition(Tss);
        present_left_foot_pose_.z = initial_left_foot_pose_.z + foot_z_traj3.getPosition(Tss);

        mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(com_x_traj32.getPosition(current_time),
                                                                  -com_y_traj32.getPosition(current_time),
                                                                  com_z_traj.getPosition(current_time), 0.0, 0.0, 0.0);

        mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(present_right_foot_pose_.x, present_right_foot_pose_.y, present_right_foot_pose_.z,
                                                                      present_right_foot_pose_.roll, present_right_foot_pose_.pitch, present_right_foot_pose_.yaw);

        mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(present_left_foot_pose_.x, present_left_foot_pose_.y, present_left_foot_pose_.z,
                                                                      present_left_foot_pose_.roll, present_left_foot_pose_.pitch, present_left_foot_pose_.yaw);
        curr_x_com = com_x_traj32.getPosition(current_time);
        curr_y_com = com_y_traj32.getPosition(current_time);
        curr_xdd_com = com_x_traj32.getAcceleration(current_time);
        curr_ydd_com = com_y_traj32.getAcceleration(current_time);

        measuredZMP_px = (measuredZMP_r_x * current_right_fz_N_ + measuredZMP_l_x * current_left_fz_N_) / (current_right_fz_N_ + current_left_fz_N_);
        measuredZMP_py = (measuredZMP_r_y * current_right_fz_N_ + measuredZMP_l_y * current_left_fz_N_) / (current_right_fz_N_ + current_left_fz_N_);

        balancing_index_ = BalancingPhase6;
      }
      else if (current_time>Tend && current_time<=(Tend+Tlift) && iswait == 0)//lifting body
      {
        present_right_foot_pose_.x = initial_right_foot_pose_.x + foot_x_traj3.getPosition(Tss);
        present_right_foot_pose_.y = initial_right_foot_pose_.y + foot_y_traj.getPosition(Tss);
        present_right_foot_pose_.z = initial_right_foot_pose_.z + foot_z_traj3.getPosition(Tss);

        present_left_foot_pose_.x = initial_left_foot_pose_.x + foot_x_traj3.getPosition(Tss);
        present_left_foot_pose_.y = initial_left_foot_pose_.y + foot_y_traj.getPosition(Tss);
        present_left_foot_pose_.z = initial_left_foot_pose_.z + foot_z_traj3.getPosition(Tss);

        mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(com_x_traj32.getPosition(Tend),
                                                                  com_y_traj32.getPosition(Tend),
                                                                  com_z_traj_end.getPosition(current_time - Tend), 0.0, 0.0, 0.0);

        mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(present_right_foot_pose_.x, present_right_foot_pose_.y, present_right_foot_pose_.z,
                                                                      present_right_foot_pose_.roll, present_right_foot_pose_.pitch, present_right_foot_pose_.yaw);

        mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(present_left_foot_pose_.x, present_left_foot_pose_.y, present_left_foot_pose_.z,
                                                                      present_left_foot_pose_.roll, present_left_foot_pose_.pitch, present_left_foot_pose_.yaw);
        curr_x_com = com_x_traj32.getPosition(current_time);
        curr_y_com = com_y_traj32.getPosition(current_time);
        curr_xdd_com = com_x_traj32.getAcceleration(current_time);
        curr_ydd_com = com_y_traj32.getAcceleration(current_time);

        balancing_index_ = BalancingPhase7;
      }

      mat_cob_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_cob_);
      mat_robot_to_rfoot_ = mat_cob_to_g_*mat_g_to_rfoot_;
      mat_robot_to_lfoot_ = mat_cob_to_g_*mat_g_to_lfoot_;

      //Balancing Algorithm
      double right_leg_fx_N  = current_right_fx_N_;
      double right_leg_fy_N  = current_right_fy_N_;
      double right_leg_fz_N  = current_right_fz_N_;
      double right_leg_Tx_Nm = current_right_tx_Nm_;
      double right_leg_Ty_Nm = current_right_ty_Nm_;
      double right_leg_Tz_Nm = current_right_tz_Nm_;

      double left_leg_fx_N  = current_left_fx_N_;
      double left_leg_fy_N  = current_left_fy_N_;
      double left_leg_fz_N  = current_left_fz_N_;
      double left_leg_Tx_Nm = current_left_tx_Nm_;
      double left_leg_Ty_Nm = current_left_ty_Nm_;
      double left_leg_Tz_Nm = current_left_tz_Nm_;

      Eigen::MatrixXd  mat_right_force, mat_right_torque;
      mat_right_force.resize(4,1);    mat_right_force.fill(0);
      mat_right_torque.resize(4,1);   mat_right_torque.fill(0);
      mat_right_force(0,0) = right_leg_fx_N;
      mat_right_force(1,0) = right_leg_fy_N;
      mat_right_force(2,0) = right_leg_fz_N;
      mat_right_torque(0,0) = right_leg_Tx_Nm;
      mat_right_torque(1,0) = right_leg_Ty_Nm;
      mat_right_torque(2,0) = right_leg_Tz_Nm;

      Eigen::MatrixXd  mat_left_force, mat_left_torque;
      mat_left_force.resize(4,1);     mat_left_force.fill(0);
      mat_left_torque.resize(4,1);    mat_left_torque.fill(0);
      mat_left_force(0,0) = left_leg_fx_N;
      mat_left_force(1,0) = left_leg_fy_N;
      mat_left_force(2,0) = left_leg_fz_N;
      mat_left_torque(0,0) = left_leg_Tx_Nm;
      mat_left_torque(1,0) = left_leg_Ty_Nm;
      mat_left_torque(2,0) = left_leg_Tz_Nm;

      mat_right_force  = mat_robot_to_rfoot_*mat_rfoot_to_rft_*mat_right_force;
      mat_right_torque = mat_robot_to_rfoot_*mat_rfoot_to_rft_*mat_right_torque;

      mat_left_force  = mat_robot_to_lfoot_*mat_lfoot_to_lft_*mat_left_force;
      mat_left_torque = mat_robot_to_lfoot_*mat_lfoot_to_lft_*mat_left_torque;

      imu_data_mutex_lock_.lock();
      double gyro_roll_rad_per_sec  = current_gyro_roll_rad_per_sec_;
      double gyro_pitch_rad_per_sec = current_gyro_pitch_rad_per_sec_;

      double iu_roll_rad  = current_imu_roll_rad_;
      double iu_pitch_rad = current_imu_pitch_rad_;
      imu_data_mutex_lock_.unlock();

      balance_ctrl_.setCurrentGyroSensorOutput(gyro_roll_rad_per_sec, gyro_pitch_rad_per_sec);
      balance_ctrl_.setCurrentOrientationSensorOutput(iu_roll_rad, iu_pitch_rad);
      balance_ctrl_.setCurrentFootForceTorqueSensorOutput(mat_right_force.coeff(0,0),  mat_right_force.coeff(1,0),  mat_right_force.coeff(2,0),
                                                          mat_right_torque.coeff(0,0), mat_right_torque.coeff(1,0), mat_right_torque.coeff(2,0),
                                                          mat_left_force.coeff(0,0),   mat_left_force.coeff(1,0),   mat_left_force.coeff(2,0),
                                                          mat_left_torque.coeff(0,0),  mat_left_torque.coeff(1,0),  mat_left_torque.coeff(2,0));

      double r_target_fx_N = 0;
      double l_target_fx_N = 0;
      double r_target_fy_N = 0;
      double l_target_fy_N = 0;
      double r_target_fz_N = right_dsp_fz_N_;
      double l_target_fz_N = left_dsp_fz_N_;

      Eigen::MatrixXd mat_g_to_acc, mat_robot_to_acc;
      mat_g_to_acc.resize(4, 1);
      mat_g_to_acc.fill(0);
      mat_g_to_acc.coeffRef(0,0) = curr_xdd_com;
      mat_g_to_acc.coeffRef(1,0) = curr_ydd_com;
      mat_robot_to_acc = mat_robot_to_g_ * mat_g_to_acc;

      switch(balancing_index_)
      {
      case BalancingPhase1:
        //fprintf(stderr, "DSP : R--O->L\n");
        r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
        r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
        r_target_fz_N = wsigmoid(current_time - TIME_UNIT, Tcshift, 0, left_ssp_fz_N_, left_dsp_fz_N_, 1.0, 1.0);
        l_target_fz_N = left_ssp_fz_N_ - l_target_fz_N;
        break;
      case BalancingPhase2:
        //fprintf(stderr, "SSP : L_BALANCING1\n");
        l_target_fx_N = 0;
        l_target_fy_N = 0;
        l_target_fz_N = 0;

        r_target_fx_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
        r_target_fy_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
        r_target_fz_N = left_ssp_fz_N_;
        break;
      case BalancingPhase3:
        //fprintf(stderr, "DSP : R--O<-L\n");
        r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
        r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
        r_target_fz_N = wsigmoid(current_time - TIME_UNIT, Tds - Tss, Tds, 0, left_ssp_fz_N_, 1.0, 1.0);
        l_target_fz_N = left_ssp_fz_N_ - l_target_fz_N;
        break;
      case BalancingPhase4:
        //fprintf(stderr, "DSP : R--O--L\n");
        r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
        r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
        l_target_fz_N = left_dsp_fz_N_;
        r_target_fz_N = right_dsp_fz_N_;
        break;
      case BalancingPhase5:
        //fprintf(stderr, "SSP : R_BALANCING1\n");
        l_target_fx_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
        l_target_fy_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
        l_target_fz_N = right_ssp_fz_N_;

        r_target_fx_N = 0;
        r_target_fy_N = 0;
        r_target_fz_N = 0;
        break;
      case BalancingPhase6:
        //fprintf(stderr, "DSP : R->O--L");
        r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
        r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
        l_target_fz_N = wsigmoid(current_time - TIME_UNIT, Tcshift, Tend - Tcshift, right_dsp_fz_N_, right_ssp_fz_N_, 1.0, 1.0);
        r_target_fz_N = left_ssp_fz_N_ - l_target_fz_N;
        break;
      case BalancingPhase7:
        //fprintf(stderr, "DSP : END");
        r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
        r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
        r_target_fz_N = right_dsp_fz_N_;
        l_target_fz_N = left_dsp_fz_N_;
        break;
      default:
        break;
      }

      balance_ctrl_.setDesiredCOBGyro(0,0);
      balance_ctrl_.setDesiredCOBOrientation(present_body_pose_.roll, present_body_pose_.pitch);
      balance_ctrl_.setDesiredFootForceTorque(r_target_fx_N*1.0, r_target_fy_N*1.0, r_target_fz_N, 0, 0, 0,
                                              l_target_fx_N*1.0, l_target_fy_N*1.0, l_target_fz_N, 0, 0, 0);
      balance_ctrl_.setDesiredPose(mat_robot_to_cob_, mat_robot_to_rfoot_, mat_robot_to_lfoot_);

      balance_ctrl_.process(&balance_error_, &mat_robot_to_cob_modified_, &mat_robot_to_rf_modified_, &mat_robot_to_lf_modified_);
      mat_cob_to_robot_modified_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_modified_);
      //Stabilizer End

      if (is_balance)
      {
        rhip_to_rfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_rhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_rf_modified_);
        lhip_to_lfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_lhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_lf_modified_);
      }
      else
      {
        rhip_to_rfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_rhip_to_cob_ * mat_cob_to_robot_) * mat_robot_to_rfoot_);
        lhip_to_lfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_lhip_to_cob_ * mat_cob_to_robot_) * mat_robot_to_lfoot_);
      }

      if(thormang3_kd_->calcInverseKinematicsForRightLeg(&r_leg_out_angle_rad_[0], rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw) == false)
      {
        printf("IK not Solved EPR : %f %f %f %f %f %f\n", rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw);
        return;
      }

      if(thormang3_kd_->calcInverseKinematicsForLeftLeg(&l_leg_out_angle_rad_[0], lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw) == false)
      {
        printf("IK not Solved EPL : %f %f %f %f %f %f\n", lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw);
        return;
      }

      for(int angle_idx = 0; angle_idx < 6; angle_idx++)/*leg*/
      {
        out_angle_rad_[angle_idx+0] = r_leg_out_angle_rad_[angle_idx];/*right leg*/
        out_angle_rad_[angle_idx+6] = l_leg_out_angle_rad_[angle_idx];/*left leg*/
      }
      // for(int angle_idx = 12; angle_idx < 19; angle_idx++)/*arm*/
      // {
      //   out_angle_rad_[angle_idx+0] = curr_angle_rad_[angle_idx+0];/*right arm*/
      //   out_angle_rad_[angle_idx+7] = curr_angle_rad_[angle_idx+7];/*left arm*/
      // }
      // out_angle_rad_[26] = curr_angle_rad_[26];/*torso*/
      // for(int angle_idx = 27; angle_idx < PROJECT_MAX_JOINT_ID+1; angle_idx++)/*gripper head*/
      // {
      //   out_angle_rad_[angle_idx] = curr_angle_rad_[angle_idx];
      // }

      if (is_walking_task == 0)
      {
        control_cycle_index = 0;//zero when there is no task in queue
      }
      else
      {
        control_cycle_index += 1;
      }
    }
    else // only balancing
    {
      //Balancing Algorithm
      curr_x_com = com_x_traj32.getPosition(Tend);
      curr_y_com = com_y_traj32.getPosition(Tend);
      curr_xdd_com = com_x_traj32.getAcceleration(Tend);
      curr_ydd_com = com_y_traj32.getAcceleration(Tend);

      double right_leg_fx_N  = current_right_fx_N_;
      double right_leg_fy_N  = current_right_fy_N_;
      double right_leg_fz_N  = current_right_fz_N_;
      double right_leg_Tx_Nm = current_right_tx_Nm_;
      double right_leg_Ty_Nm = current_right_ty_Nm_;
      double right_leg_Tz_Nm = current_right_tz_Nm_;

      double left_leg_fx_N  = current_left_fx_N_;
      double left_leg_fy_N  = current_left_fy_N_;
      double left_leg_fz_N  = current_left_fz_N_;
      double left_leg_Tx_Nm = current_left_tx_Nm_;
      double left_leg_Ty_Nm = current_left_ty_Nm_;
      double left_leg_Tz_Nm = current_left_tz_Nm_;

      Eigen::MatrixXd  mat_right_force, mat_right_torque;
      mat_right_force.resize(4,1);    mat_right_force.fill(0);
      mat_right_torque.resize(4,1);   mat_right_torque.fill(0);
      mat_right_force(0,0) = right_leg_fx_N;
      mat_right_force(1,0) = right_leg_fy_N;
      mat_right_force(2,0) = right_leg_fz_N;
      mat_right_torque(0,0) = right_leg_Tx_Nm;
      mat_right_torque(1,0) = right_leg_Ty_Nm;
      mat_right_torque(2,0) = right_leg_Tz_Nm;

      Eigen::MatrixXd  mat_left_force, mat_left_torque;
      mat_left_force.resize(4,1);     mat_left_force.fill(0);
      mat_left_torque.resize(4,1);    mat_left_torque.fill(0);
      mat_left_force(0,0) = left_leg_fx_N;
      mat_left_force(1,0) = left_leg_fy_N;
      mat_left_force(2,0) = left_leg_fz_N;
      mat_left_torque(0,0) = left_leg_Tx_Nm;
      mat_left_torque(1,0) = left_leg_Ty_Nm;
      mat_left_torque(2,0) = left_leg_Tz_Nm;

      mat_right_force  = mat_robot_to_rfoot_*mat_rfoot_to_rft_*mat_right_force;
      mat_right_torque = mat_robot_to_rfoot_*mat_rfoot_to_rft_*mat_right_torque;

      mat_left_force  = mat_robot_to_lfoot_*mat_lfoot_to_lft_*mat_left_force;
      mat_left_torque = mat_robot_to_lfoot_*mat_lfoot_to_lft_*mat_left_torque;

      imu_data_mutex_lock_.lock();
      double gyro_roll_rad_per_sec  = current_gyro_roll_rad_per_sec_;
      double gyro_pitch_rad_per_sec = current_gyro_pitch_rad_per_sec_;

      double iu_roll_rad  = current_imu_roll_rad_;
      double iu_pitch_rad = current_imu_pitch_rad_;
      imu_data_mutex_lock_.unlock();

      balance_ctrl_.setCurrentGyroSensorOutput(gyro_roll_rad_per_sec, gyro_pitch_rad_per_sec);
      balance_ctrl_.setCurrentOrientationSensorOutput(iu_roll_rad, iu_pitch_rad);
      balance_ctrl_.setCurrentFootForceTorqueSensorOutput(mat_right_force.coeff(0,0),  mat_right_force.coeff(1,0),  mat_right_force.coeff(2,0),
                                                          mat_right_torque.coeff(0,0), mat_right_torque.coeff(1,0), mat_right_torque.coeff(2,0),
                                                          mat_left_force.coeff(0,0),   mat_left_force.coeff(1,0),   mat_left_force.coeff(2,0),
                                                          mat_left_torque.coeff(0,0),  mat_left_torque.coeff(1,0),  mat_left_torque.coeff(2,0));

      double r_target_fx_N = 0;
      double l_target_fx_N = 0;
      double r_target_fy_N = 0;
      double l_target_fy_N = 0;
      double r_target_fz_N = right_dsp_fz_N_;
      double l_target_fz_N = left_dsp_fz_N_;

      Eigen::MatrixXd mat_g_to_acc, mat_robot_to_acc;
      mat_g_to_acc.resize(4, 1);
      mat_g_to_acc.fill(0);
      mat_g_to_acc.coeffRef(0,0) = curr_xdd_com;
      mat_g_to_acc.coeffRef(1,0) = curr_ydd_com;
      mat_robot_to_acc = mat_robot_to_g_ * mat_g_to_acc;

      r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      r_target_fz_N = right_dsp_fz_N_;
      l_target_fz_N = left_dsp_fz_N_;

      balance_ctrl_.setDesiredCOBGyro(0,0);
      balance_ctrl_.setDesiredCOBOrientation(present_body_pose_.roll, present_body_pose_.pitch);
      balance_ctrl_.setDesiredFootForceTorque(r_target_fx_N*1.0, r_target_fy_N*1.0, r_target_fz_N, 0, 0, 0,
                                              l_target_fx_N*1.0, l_target_fy_N*1.0, l_target_fz_N, 0, 0, 0);
      balance_ctrl_.setDesiredPose(mat_robot_to_cob_, mat_robot_to_rfoot_, mat_robot_to_lfoot_);

      balance_ctrl_.process(&balance_error_, &mat_robot_to_cob_modified_, &mat_robot_to_rf_modified_, &mat_robot_to_lf_modified_);
      mat_cob_to_robot_modified_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_modified_);
      //Stabilizer End

      rhip_to_rfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_rhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_rf_modified_);
      lhip_to_lfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_lhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_lf_modified_);

      // rhip_to_rfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_rhip_to_cob_ * mat_cob_to_robot_) * mat_robot_to_rfoot_);
      // lhip_to_lfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_lhip_to_cob_ * mat_cob_to_robot_) * mat_robot_to_lfoot_);

      if(thormang3_kd_->calcInverseKinematicsForRightLeg(&r_leg_out_angle_rad_[0], rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw) == false)
      {
        printf("IK not Solved EPR : %f %f %f %f %f %f\n", rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw);
        return;
      }

      if(thormang3_kd_->calcInverseKinematicsForLeftLeg(&l_leg_out_angle_rad_[0], lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw) == false)
      {
        printf("IK not Solved EPL : %f %f %f %f %f %f\n", lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw);
        return;
      }

      for(int angle_idx = 0; angle_idx < 6; angle_idx++)/*leg*/
      {
        out_angle_rad_[angle_idx+0] = r_leg_out_angle_rad_[angle_idx];/*right leg*/
        out_angle_rad_[angle_idx+6] = l_leg_out_angle_rad_[angle_idx];/*left leg*/
      }

    }
  }
}

double ProjectOnline::wsigmoid(double time, double period, double time_shift, double mag, double mag_shift, double sigmoid_ratio, double distortion_ratio)
{
  double value = mag_shift, Amplitude = 0.0, sigmoid_distor_gain = 1.0, t = 0.0;
  if ((sigmoid_ratio >= 1.0) && (sigmoid_ratio < 2.0))
  {
    if( time >= time_shift+period*(2-sigmoid_ratio)) {
      value = mag_shift + mag;
    }
    else
    {
      t = 2.0*M_PI*(time - time_shift)/(period*(2-sigmoid_ratio));
      sigmoid_distor_gain = distortion_ratio + (1-distortion_ratio)*(time-(time_shift+period*(1-sigmoid_ratio)))/(period*(2-sigmoid_ratio));
      Amplitude = mag/(2.0*M_PI);
      value = mag_shift + Amplitude*(t - sigmoid_distor_gain*sin(t));
    }
  }
  else if( (sigmoid_ratio >= 0.0) && (sigmoid_ratio < 1.0))
  {
    if( time <= time_shift+period*(1-sigmoid_ratio))
      value = mag_shift;
    else {
      t = 2.0*M_PI*(time - time_shift-period*(1-sigmoid_ratio))/(period*sigmoid_ratio);
      sigmoid_distor_gain = distortion_ratio + (1-distortion_ratio)*(time-(time_shift+period*(1-sigmoid_ratio)))/(period*sigmoid_ratio);
      Amplitude = mag/(2.0*M_PI);
      value = mag_shift + Amplitude*(t - sigmoid_distor_gain*sin(t));
    }
  }
  else if(( sigmoid_ratio >= 2.0) && ( sigmoid_ratio < 3.0))
  {
    double nsigmoid_ratio = sigmoid_ratio - 2.0;
    if(time <= time_shift + period*(1.0-nsigmoid_ratio)*0.5)
      value = mag_shift;
    else if(time >= time_shift + period*(1.0+nsigmoid_ratio)*0.5)
      value = mag + mag_shift;
    else {
      t = 2.0*M_PI*(time - (time_shift+period*(1.0-nsigmoid_ratio)*0.5))/(period*nsigmoid_ratio);
      sigmoid_distor_gain = distortion_ratio + (1.0-distortion_ratio)*(time-(time_shift+period*(1.0-nsigmoid_ratio)*0.5))/(period*nsigmoid_ratio);
      Amplitude = mag/(2.0*M_PI);
      value = mag_shift + Amplitude*(t - sigmoid_distor_gain*sin(t));
    }
  }
  else
    value = mag_shift;

  return value;
}

bool ProjectOnline::isRunning()
{
  return real_running;
}

void ProjectOnline::parseIniPoseData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
  {
    int     id    = it->first.as<int>();
    double  value = it->second.as<double>();

    init_joint_positions[id-1] = value * DEGREE2RADIAN;
  }
}
