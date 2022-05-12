#include <string>
#include <iostream>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <std_msgs/String.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

#include <eigen3/Eigen/Eigen>

// Eigen::VectorXd r_arm_ini_joint_positions;
// Eigen::VectorXd l_arm_ini_joint_positions;
//
// void IniJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
// {
//   for (int ix = 0; ix < msg->name.size(); ix++)
//   {
//     if (msg->name[ix] == "r_arm_sh_p1"){r_arm_ini_joint_positions(0) = msg->position[ix];}
//     else if (msg->name[ix] == "r_arm_sh_r"){r_arm_ini_joint_positions(1) = msg->position[ix];}
//     else if (msg->name[ix] == "r_arm_sh_p1"){r_arm_ini_joint_positions(2) = msg->position[ix];}
//     else if (msg->name[ix] == "r_arm_el_y"){r_arm_ini_joint_positions(3) = msg->position[ix];}
//     else if (msg->name[ix] == "r_arm_wr_r"){r_arm_ini_joint_positions(4) = msg->position[ix];}
//     else if (msg->name[ix] == "r_arm_wr_y"){r_arm_ini_joint_positions(5) = msg->position[ix];}
//     else if (msg->name[ix] == "r_arm_wr_p"){r_arm_ini_joint_positions(6) = msg->position[ix];}
//
//     else if (msg->name[ix] == "l_arm_sh_p1"){l_arm_ini_joint_positions(0) = msg->position[ix];}
//     else if (msg->name[ix] == "l_arm_sh_r"){l_arm_ini_joint_positions(1) = msg->position[ix];}
//     else if (msg->name[ix] == "l_arm_sh_p1"){l_arm_ini_joint_positions(2) = msg->position[ix];}
//     else if (msg->name[ix] == "l_arm_el_y"){l_arm_ini_joint_positions(3) = msg->position[ix];}
//     else if (msg->name[ix] == "l_arm_wr_r"){l_arm_ini_joint_positions(4) = msg->position[ix];}
//     else if (msg->name[ix] == "l_arm_wr_y"){l_arm_ini_joint_positions(5) = msg->position[ix];}
//     else if (msg->name[ix] == "l_arm_wr_p"){l_arm_ini_joint_positions(6) = msg->position[ix];}
//   }
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "thormang3_move_group_demo");
  ros::NodeHandle node_handle;

  ros::Publisher trajectory_pub = node_handle.advertise<moveit_msgs::RobotTrajectory>("/robotis/project/moveit_trajectory_msg", 1);
  ros::Publisher to_module_command_pub = node_handle.advertise<std_msgs::String>("/robotis/project/to_module_command_msg", 0);

  ros::Rate wait_rate(1);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  bool is_callback_recieved = false;

  // r_arm_ini_joint_positions = Eigen::VectorXd::Zero(7);
  // l_arm_ini_joint_positions = Eigen::VectorXd::Zero(7);

  static const std::string PLANNING_GROUP_R_ARM = "r_arm_group";
  static const std::string PLANNING_GROUP_L_ARM = "l_arm_group";

  moveit::planning_interface::MoveGroupInterface move_group_r_arm(PLANNING_GROUP_R_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_l_arm(PLANNING_GROUP_L_ARM);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  // const moveit::core::RobotModelPtr& thormang3_model = robot_model_loader.getModel();
  // ROS_INFO("Model frame: %s", thormang3_model->getModelFrame().c_str());
  // moveit::core::RobotStatePtr thormang3_robot_initial_state(new moveit::core::RobotState(thormang3_model));
  // thormang3_robot_initial_state->setToDefaultValues();
  //
  // ros::Subscriber ini_joint_state_sub = node_handle.subscribe("/robotis/thormang3/joint_states", 5, IniJointStateCallback);
  // ros::spinOnce();

  const robot_state::JointModelGroup* joint_model_group_r_arm = move_group_r_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_R_ARM);
  const robot_state::JointModelGroup* joint_model_group_l_arm = move_group_l_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_L_ARM);
  // thormang3_robot_initial_state->setJointGroupPositions(joint_model_group_r_arm, r_arm_ini_joint_positions);

  // move_group_r_arm.setStartState(*thormang3_robot_initial_state);

  move_group_r_arm.setStartState(*move_group_r_arm.getCurrentState());
  move_group_l_arm.setStartState(*move_group_l_arm.getCurrentState());

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("pelvis_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group_r_arm.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_r_arm.getEndEffectorLink().c_str());

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_r_arm;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_l_arm;

  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject kitchen_collision_object;
  kitchen_collision_object.header.frame_id = move_group_r_arm.getPlanningFrame();

  // The id of the object is used to identify it.
  kitchen_collision_object.id = "kitchen";

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose kitchen_pose;
  kitchen_pose.orientation.z = -sin(-3.14/2);
  kitchen_pose.orientation.w = cos(3.14/2);
  kitchen_pose.position.x = 0.3;
  // kitchen_pose.position.y = 0.45;
  // kitchen_pose.position.z = -0.64;
  kitchen_pose.position.y = 0.55;
  kitchen_pose.position.z = -0.92; //-0.85

  // Eigen::Vector3d kitchen_scale(0.75, 0.75, 0.75);
  shapes::Mesh* kitchen_shape = shapes::createMeshFromResource("package://thormang3_move_group/model/kitchen.stl");
  shape_msgs::Mesh kitchen_model;
  shapes::ShapeMsg kitchen_model_msg = kitchen_model;
  shapes::constructMsgFromShape(kitchen_shape,kitchen_model_msg);
  kitchen_model = boost::get<shape_msgs::Mesh>(kitchen_model_msg);

  kitchen_collision_object.meshes.push_back(kitchen_model);
  kitchen_collision_object.mesh_poses.push_back(kitchen_pose);
  kitchen_collision_object.operation = kitchen_collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> kitchen_collision_object_msg;
  kitchen_collision_object_msg.push_back(kitchen_collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(kitchen_collision_object_msg);

  moveit_msgs::CollisionObject cup_collision_object;
  cup_collision_object.header.frame_id = move_group_r_arm.getPlanningFrame();

  // The id of the object is used to identify it.
  cup_collision_object.id = "cup";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.03;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose cup_pose;
  cup_pose.orientation.w = 1.0;
  cup_pose.position.x = 0.45;
  cup_pose.position.y = -0.25;
  cup_pose.position.z = -0.03;

  cup_collision_object.primitives.push_back(primitive);
  cup_collision_object.primitive_poses.push_back(cup_pose);
  cup_collision_object.operation = cup_collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> cup_collision_object_msg;
  cup_collision_object_msg.push_back(cup_collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(cup_collision_object_msg);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Wait for MoveGroup to recieve and process the collision object message
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // Now when we plan a trajectory it will avoid the obstacle
  geometry_msgs::Pose pose1;
  pose1.orientation.x = 0;
  pose1.orientation.y = sin(1.57/2);
  pose1.orientation.z = 0;
  pose1.orientation.w = cos(1.57/2);
  pose1.position.x = 0.45;
  pose1.position.y = -0.2;
  pose1.position.z = 0.22;
  move_group_r_arm.setPoseTarget(pose1);

  bool success = (move_group_r_arm.plan(my_plan_r_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan_r_arm.trajectory_, joint_model_group_r_arm);

  // Publish planned trajectory
  trajectory_pub.publish(my_plan_r_arm.trajectory_);
  wait_rate.sleep();
  std_msgs::String moveit_control_msg;
  moveit_control_msg.data = "moveit_control_on";
  to_module_command_pub.publish(moveit_control_msg);
  ros::spinOnce();

  visual_tools.trigger();
  visual_tools.prompt("Press 'next'");

  geometry_msgs::Pose pose2;
  pose2.orientation.x = 0;
  pose2.orientation.y = sin(1.57/2);
  pose2.orientation.z = 0;
  pose2.orientation.w = cos(1.57/2);
  pose2.position.x = 0.45;
  pose2.position.y = -0.2;
  pose2.position.z = 0.18;
  move_group_r_arm.setStartState(*move_group_r_arm.getCurrentState());
  move_group_r_arm.setPoseTarget(pose2);
  // move_group_r_arm.setPlanningTime(10.0);

  success = (move_group_r_arm.plan(my_plan_r_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan_r_arm.trajectory_, joint_model_group_r_arm);

  // Publish planned trajectory
  trajectory_pub.publish(my_plan_r_arm.trajectory_);
  wait_rate.sleep();
  to_module_command_pub.publish(moveit_control_msg);
  ros::spinOnce();

  visual_tools.trigger();
  visual_tools.prompt("next step");

  // Now, let's attach the collision object to the robot.
  ROS_INFO_NAMED("tutorial", "Attach the object to the right arm");
  move_group_r_arm.attachObject(cup_collision_object.id);

  visual_tools.trigger();
  visual_tools.prompt("Next: Right arm gripper closing");

  std_msgs::String gripper_control_msg;
  gripper_control_msg.data = "r_gripper_close";
  to_module_command_pub.publish(gripper_control_msg);

  visual_tools.trigger();
  visual_tools.prompt("Next:");

  // moveit_msgs::OrientationConstraint r_arm_ocm;
  // r_arm_ocm.link_name = "r_arm_end_link";
  // r_arm_ocm.header.frame_id = "pelvis_link";
  // r_arm_ocm.orientation.x = 0;
  // r_arm_ocm.orientation.y = sin(1.57/2);
  // r_arm_ocm.orientation.z = 0;
  // r_arm_ocm.orientation.w = cos(1.57/2);
  // r_arm_ocm.absolute_x_axis_tolerance = 10;
  // r_arm_ocm.absolute_y_axis_tolerance = 10;
  // r_arm_ocm.absolute_z_axis_tolerance = 10;
  // r_arm_ocm.weight = 1.0;
  //
  // moveit_msgs::Constraints r_arm_constraints;
  // r_arm_constraints.orientation_constraints.push_back(r_arm_ocm);
  // move_group_r_arm.setPathConstraints(r_arm_constraints);

  geometry_msgs::Pose pose3_r_arm;
  pose3_r_arm.orientation.x = 0;
  pose3_r_arm.orientation.y = sin(1.57/2);
  pose3_r_arm.orientation.z = 0;
  pose3_r_arm.orientation.w = cos(1.57/2);
  pose3_r_arm.position.x = 0.38;
  pose3_r_arm.position.y = -0.1;
  pose3_r_arm.position.z = 0.4;
  move_group_r_arm.setStartState(*move_group_r_arm.getCurrentState());
  move_group_r_arm.setPoseTarget(pose3_r_arm);
  bool success_r_arm = (move_group_r_arm.plan(my_plan_r_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan_r_arm.trajectory_, joint_model_group_r_arm);

  if (success_r_arm)
  {
    trajectory_pub.publish(my_plan_r_arm.trajectory_);
    wait_rate.sleep();
    to_module_command_pub.publish(moveit_control_msg);
    ros::spinOnce();
  }
  move_group_r_arm.clearPathConstraints();

  std::vector<std::string> object_ids;
  object_ids.push_back(cup_collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  visual_tools.trigger();
  visual_tools.prompt("Next:");
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal move around cuboid) %s", (success_r_arm)  ? "" : "FAILED");

  geometry_msgs::Pose pose3_l_arm;
  pose3_l_arm.orientation.x = 0;
  pose3_l_arm.orientation.y = 0;
  pose3_l_arm.orientation.z = -sin(1.57/2);
  pose3_l_arm.orientation.w = cos(1.57/2);
  pose3_l_arm.position.x = 0.32;
  pose3_l_arm.position.y = 0.1;
  pose3_l_arm.position.z = 0.15;
  move_group_l_arm.setStartState(*move_group_l_arm.getCurrentState());
  move_group_l_arm.setPoseTarget(pose3_l_arm);
  bool success_l_arm = (move_group_l_arm.plan(my_plan_l_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan_l_arm.trajectory_, joint_model_group_l_arm);

  if (success_l_arm)
  {
    trajectory_pub.publish(my_plan_l_arm.trajectory_);
    wait_rate.sleep();
    to_module_command_pub.publish(moveit_control_msg);
    ros::spinOnce();
  }

  visual_tools.trigger();
  visual_tools.prompt("Next:");

  // moveit_msgs::OrientationConstraint l_arm_ocm;
  // l_arm_ocm.link_name = "l_arm_end_link";
  // l_arm_ocm.header.frame_id = "pelvis_link";
  // l_arm_ocm.orientation.x = 0;
  // l_arm_ocm.orientation.y = 0;
  // l_arm_ocm.orientation.z = -sin(1.57/2);
  // l_arm_ocm.orientation.w = cos(1.57/2);
  // l_arm_ocm.absolute_x_axis_tolerance = 10;
  // l_arm_ocm.absolute_y_axis_tolerance = 10;
  // l_arm_ocm.absolute_z_axis_tolerance = 10;
  // l_arm_ocm.weight = 1.0;
  //
  // moveit_msgs::Constraints l_arm_constraints;
  // l_arm_constraints.orientation_constraints.push_back(l_arm_ocm);
  // move_group_l_arm.setPathConstraints(l_arm_constraints);

  geometry_msgs::Pose pose4_l_arm;
  pose4_l_arm.orientation.x = 0;
  pose4_l_arm.orientation.y = 0;
  pose4_l_arm.orientation.z = -sin(1.57/2);
  pose4_l_arm.orientation.w = cos(1.57/2);
  pose4_l_arm.position.x = 0.32;
  pose4_l_arm.position.y = 0.02;
  pose4_l_arm.position.z = 0.15;
  move_group_l_arm.setStartState(*move_group_l_arm.getCurrentState());
  move_group_l_arm.setPoseTarget(pose4_l_arm);
  success_l_arm = (move_group_l_arm.plan(my_plan_l_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan_l_arm.trajectory_, joint_model_group_l_arm);

  if (success_l_arm)
  {
    trajectory_pub.publish(my_plan_l_arm.trajectory_);
    wait_rate.sleep();
    to_module_command_pub.publish(moveit_control_msg);
    ros::spinOnce();
  }
  move_group_l_arm.clearPathConstraints();

  // geometry_msgs::Pose pose4_r_arm;
  // pose4_r_arm.orientation.x = 0;
  // pose4_r_arm.orientation.y = sin(1.57/2);
  // pose4_r_arm.orientation.z = 0;
  // pose4_r_arm.orientation.w = cos(1.57/2);
  // pose4_r_arm.position.x = 0.38;
  // pose4_r_arm.position.y = -0.05;
  // pose4_r_arm.position.z = 0.4;
  // move_group_r_arm.setStartState(*move_group_r_arm.getCurrentState());
  // move_group_r_arm.setPoseTarget(pose4_r_arm);
  // success_r_arm = (move_group_r_arm.plan(my_plan_r_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan_l_arm.trajectory_, joint_model_group_l_arm);
  //
  // if (success_r_arm)
  // {
  //   trajectory_pub.publish(my_plan_r_arm.trajectory_);
  //   wait_rate.sleep();
  //   to_module_command_pub.publish(moveit_control_msg);
  //   ros::spinOnce();
  // }
  // move_group_r_arm.clearPathConstraints();

  visual_tools.trigger();
  visual_tools.prompt("Next: Left arm gripper closing");

  gripper_control_msg.data = "l_gripper_close";
  to_module_command_pub.publish(gripper_control_msg);

  visual_tools.trigger();
  visual_tools.prompt("Next:");

  gripper_control_msg.data = "r_gripper_open";
  to_module_command_pub.publish(gripper_control_msg);

  ROS_INFO_NAMED("tutorial", "Detach the object from the right arm");
  move_group_r_arm.detachObject(cup_collision_object.id);

  visual_tools.trigger();
  visual_tools.prompt("Next:");

  ROS_INFO_NAMED("tutorial", "Attach the object to the left arm");
  move_group_l_arm.attachObject(cup_collision_object.id);

  visual_tools.trigger();
  visual_tools.prompt("Next:");

  geometry_msgs::Pose pose5_l_arm;
  pose5_l_arm.orientation.x = 0;
  pose5_l_arm.orientation.y = 0;
  pose5_l_arm.orientation.z = sin(1.57/2);
  pose5_l_arm.orientation.w = cos(1.57/2);
  pose5_l_arm.position.x = 0.05;
  pose5_l_arm.position.y = 0.6;
  pose5_l_arm.position.z = 0.15;
  move_group_l_arm.setStartState(*move_group_l_arm.getCurrentState());
  move_group_l_arm.setPoseTarget(pose5_l_arm);
  success_l_arm = (move_group_l_arm.plan(my_plan_l_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan_r_arm.trajectory_, joint_model_group_l_arm);

  geometry_msgs::Pose pose5_r_arm;
  pose5_r_arm.orientation.x = 0;
  pose5_r_arm.orientation.y = sin(1.57/2);
  pose5_r_arm.orientation.z = 0;
  pose5_r_arm.orientation.w = cos(1.57/2);
  pose5_r_arm.position.x = 0.38;
  pose5_r_arm.position.y = -0.1;
  pose5_r_arm.position.z = 0.42;
  move_group_r_arm.setStartState(*move_group_r_arm.getCurrentState());
  move_group_r_arm.setPoseTarget(pose5_r_arm);
  success_r_arm = (move_group_r_arm.plan(my_plan_r_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan_l_arm.trajectory_, joint_model_group_l_arm);

  if (success_l_arm || success_r_arm)
  {
    trajectory_pub.publish(my_plan_l_arm.trajectory_);
    wait_rate.sleep();
    trajectory_pub.publish(my_plan_r_arm.trajectory_);
    wait_rate.sleep();
    to_module_command_pub.publish(moveit_control_msg);
    ros::spinOnce();
  }

  visual_tools.trigger();
  visual_tools.prompt("Next:");

  gripper_control_msg.data = "l_gripper_open";
  to_module_command_pub.publish(gripper_control_msg);

  ros::shutdown();
  return 0;
}
