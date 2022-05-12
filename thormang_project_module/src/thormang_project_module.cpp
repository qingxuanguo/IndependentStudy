#include <iostream>
#include <fstream>
#include <stdio.h>
#include <math.h>
#include "thormang_project_module/thormang_project_module.h"

static const double TIME_UNIT = 0.008;

const int PROJECT_MAX_JOINT_ID = 30; // STARTING FROM 0
const double MAX_JOINT_ROTATION = 0.1; // rad

double slowed_ratio = 10;

sensor_msgs::ChannelFloat32 test_data;

class ProjectStatusMSG
{
public:
  static const std::string PROJECT_MODULE_IS_ENABLED_MSG;
};

const std::string ProjectStatusMSG::PROJECT_MODULE_IS_ENABLED_MSG = "Project_Module_is_enabled";

using namespace thormang3;

ProjectModule::ProjectModule()
  : control_cycle_msec_(8)
{
  enable_              = false;
  is_online_control    = true;
  is_direct_control    = false;
  is_moveit_control    = false;
  is_moveit_executed   = true;
  is_r_gripper_control = false;
  is_l_gripper_control = false;

  control_cycle_index = 0;

  module_name_  = "project_module"; // set unique module name
  control_mode_ = robotis_framework::PositionControl;

  /* leg */
  result_["r_leg_hip_y"] = new robotis_framework::DynamixelState();
  result_["r_leg_hip_r"] = new robotis_framework::DynamixelState();
  result_["r_leg_hip_p"] = new robotis_framework::DynamixelState();
  result_["r_leg_kn_p" ] = new robotis_framework::DynamixelState();
  result_["r_leg_an_p" ] = new robotis_framework::DynamixelState();
  result_["r_leg_an_r" ] = new robotis_framework::DynamixelState();

  result_["l_leg_hip_y"] = new robotis_framework::DynamixelState();
  result_["l_leg_hip_r"] = new robotis_framework::DynamixelState();
  result_["l_leg_hip_p"] = new robotis_framework::DynamixelState();
  result_["l_leg_kn_p" ] = new robotis_framework::DynamixelState();
  result_["l_leg_an_p" ] = new robotis_framework::DynamixelState();
  result_["l_leg_an_r" ] = new robotis_framework::DynamixelState();

  /* arm */
  result_["r_arm_sh_p1"]  = new robotis_framework::DynamixelState();
  result_["r_arm_sh_r"]   = new robotis_framework::DynamixelState();
  result_["r_arm_sh_p2"]  = new robotis_framework::DynamixelState();
  result_["r_arm_el_y"]   = new robotis_framework::DynamixelState();
  result_["r_arm_wr_r"]   = new robotis_framework::DynamixelState();
  result_["r_arm_wr_y"]   = new robotis_framework::DynamixelState();
  result_["r_arm_wr_p"]   = new robotis_framework::DynamixelState();

  result_["l_arm_sh_p1"]  = new robotis_framework::DynamixelState();
  result_["l_arm_sh_r"]   = new robotis_framework::DynamixelState();
  result_["l_arm_sh_p2"]  = new robotis_framework::DynamixelState();
  result_["l_arm_el_y"]   = new robotis_framework::DynamixelState();
  result_["l_arm_wr_r"]   = new robotis_framework::DynamixelState();
  result_["l_arm_wr_y"]   = new robotis_framework::DynamixelState();
  result_["l_arm_wr_p"]   = new robotis_framework::DynamixelState();

  result_["torso_y"]      = new robotis_framework::DynamixelState();

  /* head */
  result_["head_y"]       = new robotis_framework::DynamixelState();
  result_["head_p"]       = new robotis_framework::DynamixelState();
  /* gripper */
  result_["r_arm_grip"]   = new robotis_framework::DynamixelState();
  result_["l_arm_grip"]   = new robotis_framework::DynamixelState();

  /* leg */
  joint_name_to_index_["r_leg_hip_y"] = 0;
  joint_name_to_index_["r_leg_hip_r"] = 1;
  joint_name_to_index_["r_leg_hip_p"] = 2;
  joint_name_to_index_["r_leg_kn_p" ] = 3;
  joint_name_to_index_["r_leg_an_p" ] = 4;
  joint_name_to_index_["r_leg_an_r" ] = 5;

  joint_name_to_index_["l_leg_hip_y"] = 6;
  joint_name_to_index_["l_leg_hip_r"] = 7;
  joint_name_to_index_["l_leg_hip_p"] = 8;
  joint_name_to_index_["l_leg_kn_p" ] = 9;
  joint_name_to_index_["l_leg_an_p" ] = 10;
  joint_name_to_index_["l_leg_an_r" ] = 11;

  /* arm */
  joint_name_to_index_["r_arm_sh_p1"] = 12;
  joint_name_to_index_["r_arm_sh_r"]  = 13;
  joint_name_to_index_["r_arm_sh_p2"] = 14;
  joint_name_to_index_["r_arm_el_y"]  = 15;
  joint_name_to_index_["r_arm_wr_r"]  = 16;
  joint_name_to_index_["r_arm_wr_y"]  = 17;
  joint_name_to_index_["r_arm_wr_p"]  = 18;

  joint_name_to_index_["l_arm_sh_p1"] = 19;
  joint_name_to_index_["l_arm_sh_r"]  = 20;
  joint_name_to_index_["l_arm_sh_p2"] = 21;
  joint_name_to_index_["l_arm_el_y"]  = 22;
  joint_name_to_index_["l_arm_wr_r"]  = 23;
  joint_name_to_index_["l_arm_wr_y"]  = 24;
  joint_name_to_index_["l_arm_wr_p"]  = 25;

  joint_name_to_index_["torso_y"]     = 26;

  /* head */
  joint_name_to_index_["head_y"]      = 27;
  joint_name_to_index_["head_p"]      = 28;

  /* gripper */
  joint_name_to_index_["r_arm_grip"]  = 29;
  joint_name_to_index_["l_arm_grip"]  = 30;

  gyro_roll_ = gyro_pitch_ = 0;
  orientation_roll_ = orientation_pitch_ = 0;
  r_foot_fx_N_  = r_foot_fy_N_  = r_foot_fz_N_  = 0;
  r_foot_Tx_Nm_ = r_foot_Ty_Nm_ = r_foot_Tz_Nm_ = 0;
  l_foot_fx_N_  = l_foot_fy_N_  = l_foot_fz_N_  = 0;
  l_foot_Tx_Nm_ = l_foot_Ty_Nm_ = l_foot_Tz_Nm_ = 0;
}

ProjectModule::~ProjectModule()
{
  queue_thread_.join();
}

void ProjectModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  is_online_control     = true;
  is_direct_control     = false;
  is_moveit_control     = false;
  is_moveit_executed    = true;
  is_r_gripper_control  = false;
  is_l_gripper_control  = false;

  control_cycle_index = 0;

  gripper_move_time = 1.5;
  gripper_traj_waypoints = int(gripper_move_time / TIME_UNIT) + 1;
  gripper_closed_rad = 50*3.14/180;
  gripper_open_rad = 3*3.14/180;

  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&ProjectModule::queueThread, this));
  ProjectOnline *project_online = ProjectOnline::getInstance();

  project_online->initialize();

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
      state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = project_online->out_angle_rad_[joint_name_to_index_[joint_name]];
  }

  std::string ini_pose_path = ros::package::getPath("thormang_project_module") + "/data/ini_pose.yaml";
  parseIniPoseData(ini_pose_path);

  moveit_control_traj = Eigen::MatrixXd::Zero(1000, PROJECT_MAX_JOINT_ID + 1);

  for (int joint_index = 0; joint_index <= PROJECT_MAX_JOINT_ID; joint_index++)
  {
    curr_joint_positions[joint_index] = init_joint_positions[joint_index];
    goal_joint_positions[joint_index] = init_joint_positions[joint_index];
  }

  //gyro_roll, gyro_pitch, r_foot_fx, y, z, tx, y, z, l_foot_fx, y, z, tx, y, z, ZMP_x, y
  test_data.values.resize(16);
}

void ProjectModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  to_module_command_sub    = ros_node.subscribe("/robotis/project/to_module_command_msg", 10, &ProjectModule::topicCallback, this);
  // direct_control_sub       = ros_node.subscribe("/robotis/project/joint_pose_msg", 10, &ProjectModule::directControlCallback, this);
  moveit_traj_control_sub  = ros_node.subscribe("/robotis/project/moveit_trajectory_msg", 10, &ProjectModule::moveitTrajCallback, this);
  imu_data_sub             = ros_node.subscribe("/robotis/sensor/imu/imu",                3,  &ProjectModule::imuDataOutputCallback,this);

  /* publisher */
  pub1_ = ros_node.advertise<std_msgs::Int16>("/tutorial_publish", 1, true);
  status_msg_pub     = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  data_pub           = ros_node.advertise<sensor_msgs::ChannelFloat32>("/robotis/data/data_collect", 1);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  if(ros::param::get("gazebo", gazebo_) == false)
    gazebo_ = false;

  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void ProjectModule::topicCallback(const std_msgs::String& msg)
{
  ProjectOnline *project_online = ProjectOnline::getInstance();
  if (msg.data == "phase1")
  {
    if (enable_)
    {
      project_online->phase_1_start();
    }
  }
  // else if (msg.data == "direct_control_on")
  // {
  //   is_direct_control = true;
  //   is_online_control = false;
  //   is_moveit_control = false;
  //   is_gripper_to_close = false;
  //   is_gripper_to_open  = false;
  // }
  else if (msg.data == "moveit_control_on")
  {
    is_moveit_control   = true;
    is_moveit_executed  = true;
    is_online_control   = false;
    is_direct_control   = false;
    is_r_gripper_control = false;
    is_l_gripper_control  = false;
  }
  else if (msg.data == "r_gripper_close" || msg.data == "r_gripper_open")
  {
    is_r_gripper_control = true;
    is_moveit_control    = false;
    is_online_control    = false;
    is_direct_control    = false;
    is_l_gripper_control = false;
    if (msg.data == "r_gripper_close")
    {
      r_gripper_traj.resize(gripper_traj_waypoints, 1);
      r_gripper_traj = robotis_framework::calcMinimumJerkTra(curr_joint_positions[29], 0.0, 0.0, gripper_closed_rad , 0.0 , 0.0 , TIME_UNIT, gripper_move_time);
    }
    else if (msg.data == "r_gripper_open")
    {
      r_gripper_traj.resize(gripper_traj_waypoints, 1);
      r_gripper_traj = robotis_framework::calcMinimumJerkTra(curr_joint_positions[29], 0.0, 0.0, gripper_open_rad , 0.0 , 0.0 , TIME_UNIT, gripper_move_time);
    }
  }
  else if (msg.data == "l_gripper_close" || msg.data == "l_gripper_open")
  {
    is_l_gripper_control = true;
    is_moveit_control    = false;
    is_online_control    = false;
    is_direct_control    = false;
    is_r_gripper_control = false;
    if (msg.data == "l_gripper_close")
    {
      l_gripper_traj.resize(gripper_traj_waypoints, 1);
      l_gripper_traj = robotis_framework::calcMinimumJerkTra(curr_joint_positions[30], 0.0, 0.0, gripper_closed_rad , 0.0 , 0.0 , TIME_UNIT, gripper_move_time);
    }
    else if (msg.data == "l_gripper_open")
    {
      l_gripper_traj.resize(gripper_traj_waypoints, 1);
      l_gripper_traj = robotis_framework::calcMinimumJerkTra(curr_joint_positions[30], 0.0, 0.0, gripper_open_rad , 0.0 , 0.0 , TIME_UNIT, gripper_move_time);
    }
  }
}

// void ProjectModule::directControlCallback(const sensor_msgs::JointState::ConstPtr& msg)
// {
//   if (is_direct_control == false)
//     return;
//
//   for (int ix = 0; ix < msg->name.size(); ix++)
//   {
//     goal_joint_positions[ix+12] = msg->position[ix];
//   }
// }

void ProjectModule::moveitTrajCallback(const moveit_msgs::RobotTrajectory &msg)
{
  if (enable_ == false)
    return;

  bool is_right_arm;
  if (msg.joint_trajectory.joint_names[0] == "r_arm_sh_p1" ||
      msg.joint_trajectory.joint_names[0] == "r_arm_sh_r" ||
      msg.joint_trajectory.joint_names[0] == "r_arm_sh_p2" ||
      msg.joint_trajectory.joint_names[0] == "r_arm_el_y" ||
      msg.joint_trajectory.joint_names[0] == "r_arm_wr_p" ||
      msg.joint_trajectory.joint_names[0] == "r_arm_wr_r" ||
      msg.joint_trajectory.joint_names[0] == "r_arm_wr_y")
  {
    is_right_arm = true;
  }
  else
  {
    is_right_arm = false;
  }

  if (is_moveit_executed)
  {
    moveit_traj_waypoints = msg.joint_trajectory.points.size();
    moveit_control_traj.resize(moveit_traj_waypoints, PROJECT_MAX_JOINT_ID + 1);
    moveit_traj_waypoints_slowed = moveit_traj_waypoints*slowed_ratio;
    for (int point_index = 0; point_index < moveit_traj_waypoints; point_index++)
    {
      if (is_right_arm)
      {
        for (int joint_index = 0; joint_index < msg.joint_trajectory.joint_names.size(); joint_index++)
        {
          moveit_control_traj(point_index, joint_index+12) = msg.joint_trajectory.points[point_index].positions[joint_index];//12-18
        }
        for (int joint_index2 = 0; joint_index2 < 12; joint_index2++)
        {
          moveit_control_traj(point_index, joint_index2) = curr_joint_positions[joint_index2];// 0-11
        }
        for (int joint_index2 = 19; joint_index2 <= PROJECT_MAX_JOINT_ID; joint_index2++)
        {
          moveit_control_traj(point_index, joint_index2) = curr_joint_positions[joint_index2];//19-30
        }
      }
      else
      {
        for (int joint_index = 0; joint_index < msg.joint_trajectory.joint_names.size(); joint_index++)
        {
          moveit_control_traj(point_index, joint_index+19) = msg.joint_trajectory.points[point_index].positions[joint_index];//19-25
        }
        for (int joint_index2 = 0; joint_index2 < 19; joint_index2++)
        {
          moveit_control_traj(point_index, joint_index2) = curr_joint_positions[joint_index2];// 0-18
        }
        for (int joint_index2 = 26; joint_index2 <= PROJECT_MAX_JOINT_ID; joint_index2++)
        {
          moveit_control_traj(point_index, joint_index2) = curr_joint_positions[joint_index2];//26-30
        }
      }
    }
  }
  else
  {
    if (moveit_traj_waypoints <= msg.joint_trajectory.points.size())  // Check if the last traj size larger than the new one
    {
      Eigen::MatrixXd temp = moveit_control_traj;
      moveit_control_traj.resize(msg.joint_trajectory.points.size(), PROJECT_MAX_JOINT_ID + 1);
      moveit_control_traj.block(0, 0, moveit_traj_waypoints, PROJECT_MAX_JOINT_ID + 1) = temp;
      for (int point_index = 0; point_index < msg.joint_trajectory.points.size(); point_index++)
      {
        if (is_right_arm)
        {
          for (int joint_index = 0; joint_index < msg.joint_trajectory.joint_names.size(); joint_index++)
          {
            moveit_control_traj(point_index, joint_index+12) = msg.joint_trajectory.points[point_index].positions[joint_index];//12-18
          }
        }
        else
        {
          for (int joint_index = 0; joint_index < msg.joint_trajectory.joint_names.size(); joint_index++)
          {
            moveit_control_traj(point_index, joint_index+19) = msg.joint_trajectory.points[point_index].positions[joint_index];//19-25
          }
        }
      }
      for (int point_index = moveit_traj_waypoints; point_index < msg.joint_trajectory.points.size(); point_index++)
      {
        if (is_right_arm)
        {
          for (int joint_index2 = 0; joint_index2 < 12; joint_index2++)
          {
            moveit_control_traj(point_index, joint_index2) = curr_joint_positions[joint_index2];// 0-11
          }
          for (int joint_index2 = 19; joint_index2 <= 25; joint_index2++)
          {
            moveit_control_traj(point_index, joint_index2) = moveit_control_traj(moveit_traj_waypoints - 1, joint_index2);//19-25
          }
          for (int joint_index2 = 26; joint_index2 <= PROJECT_MAX_JOINT_ID; joint_index2++)
          {
            moveit_control_traj(point_index, joint_index2) = curr_joint_positions[joint_index2];//26-30
          }
        }
        else
        {
          for (int joint_index2 = 0; joint_index2 < 12; joint_index2++)
          {
            moveit_control_traj(point_index, joint_index2) = curr_joint_positions[joint_index2];// 0-18
          }
          for (int joint_index2 = 12; joint_index2 <= 18; joint_index2++)
          {
            moveit_control_traj(point_index, joint_index2) = moveit_control_traj(moveit_traj_waypoints - 1, joint_index2);//19-25
          }
          for (int joint_index2 = 26; joint_index2 <= PROJECT_MAX_JOINT_ID; joint_index2++)
          {
            moveit_control_traj(point_index, joint_index2) = curr_joint_positions[joint_index2];//26-30
          }
        }
      }
      moveit_traj_waypoints = msg.joint_trajectory.points.size();
      moveit_traj_waypoints_slowed = moveit_traj_waypoints*slowed_ratio;
    }
    else
    {
      for (int point_index = 0; point_index < msg.joint_trajectory.points.size(); point_index++)
      {
        if (is_right_arm)
        {
          for (int joint_index = 0; joint_index < msg.joint_trajectory.joint_names.size(); joint_index++)
          {
            moveit_control_traj(point_index, joint_index+12) = msg.joint_trajectory.points[point_index].positions[joint_index];//12-18
          }
        }
        else
        {
          for (int joint_index = 0; joint_index < msg.joint_trajectory.joint_names.size(); joint_index++)
          {
            moveit_control_traj(point_index, joint_index+19) = msg.joint_trajectory.points[point_index].positions[joint_index];//19-25
          }
        }
      }
      for (int point_index = msg.joint_trajectory.points.size(); point_index < moveit_traj_waypoints; point_index++)
      {
        if (is_right_arm)
        {
          for (int joint_index = 0; joint_index < msg.joint_trajectory.joint_names.size(); joint_index++)
          {
            moveit_control_traj(point_index, joint_index+12) = msg.joint_trajectory.points[msg.joint_trajectory.points.size() - 1].positions[joint_index];//12-18
          }
        }
        else
        {
          for (int joint_index = 0; joint_index < msg.joint_trajectory.joint_names.size(); joint_index++)
          {
            moveit_control_traj(point_index, joint_index+19) = msg.joint_trajectory.points[msg.joint_trajectory.points.size() - 1].positions[joint_index];//19-25
          }
        }
      }
    }
  }
  is_moveit_executed = false;
}

void ProjectModule::imuDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  ProjectOnline *project_online = ProjectOnline::getInstance();

  project_online->setCurrentIMUSensorOutput(-1.0*(msg->angular_velocity.x), -1.0*(msg->angular_velocity.y),
                                            msg->orientation.x, msg->orientation.y, msg->orientation.z,
                                            msg->orientation.w);

  double current_gyro_roll_rad_per_sec_  = -1.0*(msg->angular_velocity.x);
  double current_gyro_pitch_rad_per_sec_ = -1.0*(msg->angular_velocity.y);

  Eigen::Quaterniond quat_current_imu_ = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

  Eigen::MatrixXd rot_x_pi_3d_ = robotis_framework::getRotationX(M_PI);
  Eigen::MatrixXd rot_z_pi_3d_ = robotis_framework::getRotationZ(M_PI);

  Eigen::MatrixXd mat_current_imu_ = (rot_x_pi_3d_ * quat_current_imu_.toRotationMatrix()) * rot_z_pi_3d_;

  imu_roll_rad  = atan2( mat_current_imu_.coeff(2,1), mat_current_imu_.coeff(2,2));
  imu_pitch_rad = atan2(-mat_current_imu_.coeff(2,0), sqrt(robotis_framework::powDI(mat_current_imu_.coeff(2,1), 2) + robotis_framework::powDI(mat_current_imu_.coeff(2,2), 2)));
}

void ProjectModule::onModuleEnable()
{
  std::string status_msg = ProjectStatusMSG::PROJECT_MODULE_IS_ENABLED_MSG;
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
}

void ProjectModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Project";
  status_msg.status_msg = msg;

  status_msg_pub.publish(status_msg);
}

void ProjectModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                   std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  ProjectOnline *project_online = ProjectOnline::getInstance();

  r_foot_fx_N_  = sensors["r_foot_fx_scaled_N"];
  r_foot_fy_N_  = sensors["r_foot_fy_scaled_N"];
  r_foot_fz_N_  = sensors["r_foot_fz_scaled_N"];
  r_foot_Tx_Nm_ = sensors["r_foot_tx_scaled_Nm"];
  r_foot_Ty_Nm_ = sensors["r_foot_ty_scaled_Nm"];
  r_foot_Tz_Nm_ = sensors["r_foot_tz_scaled_Nm"];

  l_foot_fx_N_  = sensors["l_foot_fx_scaled_N"];
  l_foot_fy_N_  = sensors["l_foot_fy_scaled_N"];
  l_foot_fz_N_  = sensors["l_foot_fz_scaled_N"];
  l_foot_Tx_Nm_ = sensors["l_foot_tx_scaled_Nm"];
  l_foot_Ty_Nm_ = sensors["l_foot_ty_scaled_Nm"];
  l_foot_Tz_Nm_ = sensors["l_foot_tz_scaled_Nm"];


  r_foot_fx_N_ = robotis_framework::sign(r_foot_fx_N_) * fmin( fabs(r_foot_fx_N_), 2000.0);
  r_foot_fy_N_ = robotis_framework::sign(r_foot_fy_N_) * fmin( fabs(r_foot_fy_N_), 2000.0);
  r_foot_fz_N_ = robotis_framework::sign(r_foot_fz_N_) * fmin( fabs(r_foot_fz_N_), 2000.0);
  r_foot_Tx_Nm_ = robotis_framework::sign(r_foot_Tx_Nm_) *fmin(fabs(r_foot_Tx_Nm_), 300.0);
  r_foot_Ty_Nm_ = robotis_framework::sign(r_foot_Ty_Nm_) *fmin(fabs(r_foot_Ty_Nm_), 300.0);
  r_foot_Tz_Nm_ = robotis_framework::sign(r_foot_Tz_Nm_) *fmin(fabs(r_foot_Tz_Nm_), 300.0);

  l_foot_fx_N_ = robotis_framework::sign(l_foot_fx_N_) * fmin( fabs(l_foot_fx_N_), 2000.0);
  l_foot_fy_N_ = robotis_framework::sign(l_foot_fy_N_) * fmin( fabs(l_foot_fy_N_), 2000.0);
  l_foot_fz_N_ = robotis_framework::sign(l_foot_fz_N_) * fmin( fabs(l_foot_fz_N_), 2000.0);
  l_foot_Tx_Nm_ = robotis_framework::sign(l_foot_Tx_Nm_) *fmin(fabs(l_foot_Tx_Nm_), 300.0);
  l_foot_Ty_Nm_ = robotis_framework::sign(l_foot_Ty_Nm_) *fmin(fabs(l_foot_Ty_Nm_), 300.0);
  l_foot_Tz_Nm_ = robotis_framework::sign(l_foot_Tz_Nm_) *fmin(fabs(l_foot_Tz_Nm_), 300.0);

  project_online->current_right_fx_N_  = r_foot_fx_N_;
  project_online->current_right_fy_N_  = r_foot_fy_N_;
  project_online->current_right_fz_N_  = r_foot_fz_N_;
  project_online->current_right_tx_Nm_ = r_foot_Tx_Nm_;
  project_online->current_right_ty_Nm_ = r_foot_Ty_Nm_;
  project_online->current_right_tz_Nm_ = r_foot_Tz_Nm_;

  project_online->current_left_fx_N_  = l_foot_fx_N_;
  project_online->current_left_fy_N_  = l_foot_fy_N_;
  project_online->current_left_fz_N_  = l_foot_fz_N_;
  project_online->current_left_tx_Nm_ = l_foot_Tx_Nm_;
  project_online->current_left_ty_Nm_ = l_foot_Ty_Nm_;
  project_online->current_left_tz_Nm_ = l_foot_Tz_Nm_;

  project_online->curr_angle_rad_[0]  = result_["r_leg_hip_y"]->goal_position_;
  project_online->curr_angle_rad_[1]  = result_["r_leg_hip_r"]->goal_position_;
  project_online->curr_angle_rad_[2]  = result_["r_leg_hip_p"]->goal_position_;
  project_online->curr_angle_rad_[3]  = result_["r_leg_kn_p" ]->goal_position_;
  project_online->curr_angle_rad_[4]  = result_["r_leg_an_p" ]->goal_position_;
  project_online->curr_angle_rad_[5]  = result_["r_leg_an_r" ]->goal_position_;

  project_online->curr_angle_rad_[6]  = result_["l_leg_hip_y"]->goal_position_;
  project_online->curr_angle_rad_[7]  = result_["l_leg_hip_r"]->goal_position_;
  project_online->curr_angle_rad_[8]  = result_["l_leg_hip_p"]->goal_position_;
  project_online->curr_angle_rad_[9]  = result_["l_leg_kn_p" ]->goal_position_;
  project_online->curr_angle_rad_[10] = result_["l_leg_an_p" ]->goal_position_;
  project_online->curr_angle_rad_[11] = result_["l_leg_an_r" ]->goal_position_;

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin(); state_iter != result_.end();
      state_iter++)
  {
    // int32_t p_pos = dxls[state_iter->first]->dxl_state_->present_position_;
    // int32_t g_pos = dxls[state_iter->first]->dxl_state_->goal_position_;
    project_online->curr_angle_rad_[joint_name_to_index_[state_iter->first]] = dxls[state_iter->first]->dxl_state_->present_position_;
    curr_joint_positions[joint_name_to_index_[state_iter->first]] = dxls[state_iter->first]->dxl_state_->present_position_;
  }

  project_online->process();

  // ...
  if (is_online_control == true)
  {
    if (fabs(result_["r_leg_hip_y"]->goal_position_ - project_online->out_angle_rad_[0]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_leg_hip_r"]->goal_position_ - project_online->out_angle_rad_[1]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_leg_hip_p"]->goal_position_ - project_online->out_angle_rad_[2]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_leg_kn_p" ]->goal_position_ - project_online->out_angle_rad_[3]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_leg_an_p" ]->goal_position_ - project_online->out_angle_rad_[4]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_leg_an_r" ]->goal_position_ - project_online->out_angle_rad_[5]) < MAX_JOINT_ROTATION &&

        fabs(result_["l_leg_hip_y"]->goal_position_ - project_online->out_angle_rad_[6]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_leg_hip_r"]->goal_position_ - project_online->out_angle_rad_[7]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_leg_hip_p"]->goal_position_ - project_online->out_angle_rad_[8]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_leg_kn_p" ]->goal_position_ - project_online->out_angle_rad_[9]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_leg_an_p" ]->goal_position_ - project_online->out_angle_rad_[10]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_leg_an_r" ]->goal_position_ - project_online->out_angle_rad_[11]) < MAX_JOINT_ROTATION &&

        fabs(result_["r_arm_sh_p1"]->goal_position_ - project_online->out_angle_rad_[12]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_arm_sh_r" ]->goal_position_ - project_online->out_angle_rad_[13]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_arm_sh_p2"]->goal_position_ - project_online->out_angle_rad_[14]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_arm_el_y" ]->goal_position_ - project_online->out_angle_rad_[15]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_arm_wr_r" ]->goal_position_ - project_online->out_angle_rad_[16]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_arm_wr_y" ]->goal_position_ - project_online->out_angle_rad_[17]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_arm_wr_p" ]->goal_position_ - project_online->out_angle_rad_[18]) < MAX_JOINT_ROTATION &&

        fabs(result_["l_arm_sh_p1"]->goal_position_ - project_online->out_angle_rad_[19]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_arm_sh_r" ]->goal_position_ - project_online->out_angle_rad_[20]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_arm_sh_p2"]->goal_position_ - project_online->out_angle_rad_[21]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_arm_el_y" ]->goal_position_ - project_online->out_angle_rad_[22]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_arm_wr_r" ]->goal_position_ - project_online->out_angle_rad_[23]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_arm_wr_y" ]->goal_position_ - project_online->out_angle_rad_[24]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_arm_wr_p" ]->goal_position_ - project_online->out_angle_rad_[25]) < MAX_JOINT_ROTATION &&

        fabs(result_["torso_y"    ]->goal_position_ - project_online->out_angle_rad_[26]) < MAX_JOINT_ROTATION)
    {
      result_["r_leg_hip_y"]->goal_position_ = project_online->out_angle_rad_[0];
      result_["r_leg_hip_r"]->goal_position_ = project_online->out_angle_rad_[1];
      result_["r_leg_hip_p"]->goal_position_ = project_online->out_angle_rad_[2];
      result_["r_leg_kn_p" ]->goal_position_ = project_online->out_angle_rad_[3];
      result_["r_leg_an_p" ]->goal_position_ = project_online->out_angle_rad_[4];
      result_["r_leg_an_r" ]->goal_position_ = project_online->out_angle_rad_[5];

      result_["l_leg_hip_y"]->goal_position_ = project_online->out_angle_rad_[6];
      result_["l_leg_hip_r"]->goal_position_ = project_online->out_angle_rad_[7];
      result_["l_leg_hip_p"]->goal_position_ = project_online->out_angle_rad_[8];
      result_["l_leg_kn_p" ]->goal_position_ = project_online->out_angle_rad_[9];
      result_["l_leg_an_p" ]->goal_position_ = project_online->out_angle_rad_[10];
      result_["l_leg_an_r" ]->goal_position_ = project_online->out_angle_rad_[11];

      result_["r_arm_sh_p1"]->goal_position_ = project_online->out_angle_rad_[12];
      result_["r_arm_sh_r" ]->goal_position_ = project_online->out_angle_rad_[13];
      result_["r_arm_sh_p2"]->goal_position_ = project_online->out_angle_rad_[14];
      result_["r_arm_el_y" ]->goal_position_ = project_online->out_angle_rad_[15];
      result_["r_arm_wr_r" ]->goal_position_ = project_online->out_angle_rad_[16];
      result_["r_arm_wr_y" ]->goal_position_ = project_online->out_angle_rad_[17];
      result_["r_arm_wr_p" ]->goal_position_ = project_online->out_angle_rad_[18];

      result_["l_arm_sh_p1"]->goal_position_ = project_online->out_angle_rad_[19];
      result_["l_arm_sh_r" ]->goal_position_ = project_online->out_angle_rad_[20];
      result_["l_arm_sh_p2"]->goal_position_ = project_online->out_angle_rad_[21];
      result_["l_arm_el_y" ]->goal_position_ = project_online->out_angle_rad_[22];
      result_["l_arm_wr_r" ]->goal_position_ = project_online->out_angle_rad_[23];
      result_["l_arm_wr_y" ]->goal_position_ = project_online->out_angle_rad_[24];
      result_["l_arm_wr_p" ]->goal_position_ = project_online->out_angle_rad_[25];

      result_["torso_y"    ]->goal_position_ = project_online->out_angle_rad_[26];
    }
  }
  else if (is_moveit_control)
  {
    for (int joint_index = 0; joint_index <= PROJECT_MAX_JOINT_ID; joint_index++)
    {
      goal_joint_positions[joint_index] = moveit_control_traj(control_cycle_index/slowed_ratio, joint_index);
    }

    //online_balance
    if (fabs(result_["r_leg_hip_y"]->goal_position_ - project_online->out_angle_rad_[0]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_leg_hip_r"]->goal_position_ - project_online->out_angle_rad_[1]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_leg_hip_p"]->goal_position_ - project_online->out_angle_rad_[2]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_leg_kn_p" ]->goal_position_ - project_online->out_angle_rad_[3]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_leg_an_p" ]->goal_position_ - project_online->out_angle_rad_[4]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_leg_an_r" ]->goal_position_ - project_online->out_angle_rad_[5]) < MAX_JOINT_ROTATION &&

        fabs(result_["l_leg_hip_y"]->goal_position_ - project_online->out_angle_rad_[6]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_leg_hip_r"]->goal_position_ - project_online->out_angle_rad_[7]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_leg_hip_p"]->goal_position_ - project_online->out_angle_rad_[8]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_leg_kn_p" ]->goal_position_ - project_online->out_angle_rad_[9]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_leg_an_p" ]->goal_position_ - project_online->out_angle_rad_[10]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_leg_an_r" ]->goal_position_ - project_online->out_angle_rad_[11]) < MAX_JOINT_ROTATION)
    {
      result_["r_leg_hip_y"]->goal_position_ = project_online->out_angle_rad_[0];
      result_["r_leg_hip_r"]->goal_position_ = project_online->out_angle_rad_[1];
      result_["r_leg_hip_p"]->goal_position_ = project_online->out_angle_rad_[2];
      result_["r_leg_kn_p" ]->goal_position_ = project_online->out_angle_rad_[3];
      result_["r_leg_an_p" ]->goal_position_ = project_online->out_angle_rad_[4];
      result_["r_leg_an_r" ]->goal_position_ = project_online->out_angle_rad_[5];

      result_["l_leg_hip_y"]->goal_position_ = project_online->out_angle_rad_[6];
      result_["l_leg_hip_r"]->goal_position_ = project_online->out_angle_rad_[7];
      result_["l_leg_hip_p"]->goal_position_ = project_online->out_angle_rad_[8];
      result_["l_leg_kn_p" ]->goal_position_ = project_online->out_angle_rad_[9];
      result_["l_leg_an_p" ]->goal_position_ = project_online->out_angle_rad_[10];
      result_["l_leg_an_r" ]->goal_position_ = project_online->out_angle_rad_[11];
    }

    if (fabs(result_["r_arm_sh_p1"]->goal_position_ - goal_joint_positions[12]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_arm_sh_r" ]->goal_position_ - goal_joint_positions[13]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_arm_sh_p2"]->goal_position_ - goal_joint_positions[14]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_arm_el_y" ]->goal_position_ - goal_joint_positions[15]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_arm_wr_r" ]->goal_position_ - goal_joint_positions[16]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_arm_wr_y" ]->goal_position_ - goal_joint_positions[17]) < MAX_JOINT_ROTATION &&
        fabs(result_["r_arm_wr_p" ]->goal_position_ - goal_joint_positions[18]) < MAX_JOINT_ROTATION &&

        fabs(result_["l_arm_sh_p1"]->goal_position_ - goal_joint_positions[19]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_arm_sh_r" ]->goal_position_ - goal_joint_positions[20]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_arm_sh_p2"]->goal_position_ - goal_joint_positions[21]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_arm_el_y" ]->goal_position_ - goal_joint_positions[22]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_arm_wr_r" ]->goal_position_ - goal_joint_positions[23]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_arm_wr_y" ]->goal_position_ - goal_joint_positions[24]) < MAX_JOINT_ROTATION &&
        fabs(result_["l_arm_wr_p" ]->goal_position_ - goal_joint_positions[25]) < MAX_JOINT_ROTATION)
    {

      // ROS_INFO("r_arm_sh_p1 : %f", goal_joint_positions[12]);
      // ROS_INFO("r_arm_sh_r  : %f", goal_joint_positions[13]);
      // ROS_INFO("r_arm_sh_p2 : %f", goal_joint_positions[14]);
      // ROS_INFO("r_arm_el_y  : %f", goal_joint_positions[15]);
      // ROS_INFO("r_arm_wr_r  : %f", goal_joint_positions[16]);
      // ROS_INFO("r_arm_wr_y  : %f", goal_joint_positions[17]);
      // ROS_INFO("r_arm_wr_p  : %f", goal_joint_positions[18]);
      //
      // ROS_INFO("l_arm_sh_p1 : %f", goal_joint_positions[19]);
      // ROS_INFO("l_arm_sh_r  : %f", goal_joint_positions[20]);
      // ROS_INFO("l_arm_sh_p2 : %f", goal_joint_positions[21]);
      // ROS_INFO("l_arm_el_y  : %f", goal_joint_positions[22]);
      // ROS_INFO("l_arm_wr_r  : %f", goal_joint_positions[23]);
      // ROS_INFO("l_arm_wr_y  : %f", goal_joint_positions[24]);
      // ROS_INFO("l_arm_wr_p  : %f", goal_joint_positions[25]);

      // result_["r_leg_hip_y"]->goal_position_ = goal_joint_positions[0];
      // result_["r_leg_hip_r"]->goal_position_ = goal_joint_positions[1];
      // result_["r_leg_hip_p"]->goal_position_ = goal_joint_positions[2];
      // result_["r_leg_kn_p" ]->goal_position_ = goal_joint_positions[3];
      // result_["r_leg_an_p" ]->goal_position_ = goal_joint_positions[4];
      // result_["r_leg_an_r" ]->goal_position_ = goal_joint_positions[5];
      //
      // result_["l_leg_hip_y"]->goal_position_ = goal_joint_positions[6];
      // result_["l_leg_hip_r"]->goal_position_ = goal_joint_positions[7];
      // result_["l_leg_hip_p"]->goal_position_ = goal_joint_positions[8];
      // result_["l_leg_kn_p" ]->goal_position_ = goal_joint_positions[9];
      // result_["l_leg_an_p" ]->goal_position_ = goal_joint_positions[10];
      // result_["l_leg_an_r" ]->goal_position_ = goal_joint_positions[11];

      result_["r_arm_sh_p1"]->goal_position_ = goal_joint_positions[12];
      result_["r_arm_sh_r" ]->goal_position_ = goal_joint_positions[13];
      result_["r_arm_sh_p2"]->goal_position_ = goal_joint_positions[14];
      result_["r_arm_el_y" ]->goal_position_ = goal_joint_positions[15];
      result_["r_arm_wr_r" ]->goal_position_ = goal_joint_positions[16];
      result_["r_arm_wr_y" ]->goal_position_ = goal_joint_positions[17];
      result_["r_arm_wr_p" ]->goal_position_ = goal_joint_positions[18];

      result_["l_arm_sh_p1"]->goal_position_ = goal_joint_positions[19];
      result_["l_arm_sh_r" ]->goal_position_ = goal_joint_positions[20];
      result_["l_arm_sh_p2"]->goal_position_ = goal_joint_positions[21];
      result_["l_arm_el_y" ]->goal_position_ = goal_joint_positions[22];
      result_["l_arm_wr_r" ]->goal_position_ = goal_joint_positions[23];
      result_["l_arm_wr_y" ]->goal_position_ = goal_joint_positions[24];
      result_["l_arm_wr_p" ]->goal_position_ = goal_joint_positions[25];

      // result_["torso_y"    ]->goal_position_ = goal_joint_positions[26];

      control_cycle_index++;
      if (control_cycle_index >= moveit_traj_waypoints_slowed || is_moveit_control == false)
      {
        is_moveit_control = false;
        is_moveit_executed = true;
        control_cycle_index = 0;
      }
    }
  }
  else if (is_r_gripper_control)
  {
    result_["r_arm_grip" ]->goal_position_ = r_gripper_traj(control_cycle_index, 0);
    control_cycle_index++;
    if (control_cycle_index >= gripper_traj_waypoints || is_r_gripper_control == false)
    {
      is_r_gripper_control = false;
      control_cycle_index = 0;
    }
  }
  else if (is_l_gripper_control)
  {
    result_["l_arm_grip" ]->goal_position_ = l_gripper_traj(control_cycle_index, 0);
    control_cycle_index++;
    if (control_cycle_index >= gripper_traj_waypoints || is_l_gripper_control == false)
    {
      is_l_gripper_control = false;
      control_cycle_index = 0;
    }
  }

  // data gathering
  test_data.values[0] = imu_roll_rad;
  test_data.values[1] = imu_pitch_rad;
  test_data.values[2] = r_foot_fx_N_;
  test_data.values[3] = r_foot_fy_N_;
  test_data.values[4] = r_foot_fz_N_;
  test_data.values[5] = r_foot_Tx_Nm_;
  test_data.values[6] = r_foot_Ty_Nm_;
  test_data.values[7] = r_foot_Tz_Nm_;
  test_data.values[8] = l_foot_fx_N_;
  test_data.values[9] = l_foot_fy_N_;
  test_data.values[10] = l_foot_fz_N_;
  test_data.values[11] = l_foot_Tx_Nm_;
  test_data.values[12] = l_foot_Ty_Nm_;
  test_data.values[13] = l_foot_Tz_Nm_;
  test_data.values[14] = project_online->measuredZMP_px;
  test_data.values[15] = project_online->measuredZMP_py;

  data_pub.publish(test_data);
}

void ProjectModule::stop()
{
  return;
}

bool ProjectModule::isRunning()
{
  // return false;
  return ProjectOnline::getInstance()->isRunning();
}

void ProjectModule::parseIniPoseData(const std::string &path)
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
