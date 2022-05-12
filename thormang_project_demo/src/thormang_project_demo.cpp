#include "thormang_project_demo/thormang_project_demo.h"

ros::Publisher      g_wholebody_ini_pose_pub;
ros::Publisher      g_enable_ctrl_module_pub;
ros::Publisher      g_to_module_command_pub;

ros::Subscriber     g_project_module_status_msg_sub;

void projectModuleStatusMSGCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
  ROS_INFO_STREAM("[" << msg->module_name <<"] : " << msg->status_msg);
}

void initialize()
{
  ros::NodeHandle nh;

  g_wholebody_ini_pose_pub        = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  g_enable_ctrl_module_pub        = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  g_to_module_command_pub         = nh.advertise<std_msgs::String>("/robotis/project/to_module_command_msg", 0);

  g_project_module_status_msg_sub = nh.subscribe("/robotis/status", 10, projectModuleStatusMSGCallback);
}

void moveToInitPose()
{
  std_msgs::String str_msg;
  str_msg.data = "ini_pose";

  g_wholebody_ini_pose_pub.publish( str_msg );
}

void setCtrlModule()
{
  std_msgs::String set_ctrl_mode_msg;
  set_ctrl_mode_msg.data = "project_module";
  g_enable_ctrl_module_pub.publish( set_ctrl_mode_msg );
}

void phaseOne()
{
  std_msgs::String demo_to_module_msg;
  demo_to_module_msg.data = "phase1";
  g_to_module_command_pub.publish(demo_to_module_msg);
}
