#include "thormang_project_demo/thormang_project_demo.h"

ros::Subscriber g_demo_command_sub;

bool is_init_pose = false;

void demoCommandCallback(const std_msgs::String::ConstPtr& msg)
{

  ROS_INFO_STREAM("[Demo]  : receive [" << msg->data << "] msg " );

  if(msg->data == "ini_pose")
  {
    ROS_INFO("go to initial pose");
    moveToInitPose();
    is_init_pose = true;
    ROS_INFO("[Demo]  : please wait 5 seconds");
  }
  else if ( msg->data == "set_mode")
  {
    ROS_INFO("set whole body control mode");
    setCtrlModule();
  }
  else if( msg->data == "phase1" )
  {
    ROS_INFO("phase 1");
    phaseOne();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thormang_project_demo");

  ROS_INFO("THORMANG Project Demo");

  initialize();

  ros::NodeHandle nh;
  g_demo_command_sub = nh.subscribe("/robotis/project_demo/command", 10, demoCommandCallback);

  ros::spin();
  return 0;
}

//rostopic pub -1 /robotis/project_demo/command std_msgs/String "ini_pose"
