#ifndef THORMANG_PROJECT_DEMO_THORMANG_PROJECT_DEMO_H_
#define THORMANG_PROJECT_DEMO_THORMANG_PROJECT_DEMO_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>

#include "robotis_controller_msgs/StatusMsg.h"

void initialize();

void moveToInitPose();

void setCtrlModule();

void phaseOne();

#endif
