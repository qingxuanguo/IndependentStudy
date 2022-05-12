#ifndef THORMANG_PROJECT_MODULE_THORMANG_PROJECT_MODULE_H_
#define THORMANG_PROJECT_MODULE_THORMANG_PROJECT_MODULE_H_

#include <ros/ros.h>
#include <unistd.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "thormang_project_module/project_online.h"

#include "robotis_math/robotis_math.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_framework_common/motion_module.h"

namespace thormang3
{

class ProjectModule
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<ProjectModule>
{
private:
  int           control_cycle_msec_;
  boost::thread queue_thread_;

  bool            gazebo_;
  bool            is_online_control;
  bool            is_direct_control;
  bool            is_moveit_control;
  bool            is_moveit_executed;
  bool            is_r_gripper_control;
  bool            is_l_gripper_control;

  /* sample subscriber & publisher */
  ros::Subscriber to_module_command_sub;
  // ros::Subscriber direct_control_joint_sub;
  ros::Subscriber moveit_traj_control_sub;
  ros::Subscriber imu_data_sub;
  ros::Publisher  pub1_;
  ros::Publisher  status_msg_pub;
  ros::Publisher  data_pub;

  void queueThread();
  void publishStatusMsg(unsigned int type, std::string msg);
  void parseIniPoseData(const std::string &path);

  void imuDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg);

  std::map<std::string, int> joint_name_to_index_;

  double init_joint_positions[31];
  double curr_joint_positions[31];
  double goal_joint_positions[31];
  Eigen::MatrixXd moveit_control_traj;
  Eigen::MatrixXd l_gripper_traj;
  Eigen::MatrixXd r_gripper_traj;

  int moveit_traj_waypoints;
  int moveit_traj_waypoints_slowed;
  int control_cycle_index;
  int gripper_traj_waypoints;

  double gripper_move_time;
  double gripper_closed_rad;
  double gripper_open_rad;

  double imu_roll_rad, imu_pitch_rad;
public:
  ProjectModule();
  virtual ~ProjectModule();

  /* ROS Topic Callback Functions */
  void topicCallback(const std_msgs::String& msg);
  void directControlCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void moveitTrajCallback(const moveit_msgs::RobotTrajectory &msg);

  void onModuleEnable();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  double gyro_roll_, gyro_pitch_;
  double orientation_roll_, orientation_pitch_;
  double r_foot_fx_N_,  r_foot_fy_N_,  r_foot_fz_N_;
  double r_foot_Tx_Nm_, r_foot_Ty_Nm_, r_foot_Tz_Nm_;
  double l_foot_fx_N_,  l_foot_fy_N_,  l_foot_fz_N_;
  double l_foot_Tx_Nm_, l_foot_Ty_Nm_, l_foot_Tz_Nm_;
};

}

#endif
