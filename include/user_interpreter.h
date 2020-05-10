// Standard import
// Writtin by Jonathan Österberg
// For questions please contact: 
//                   Jonte_ost@live.com

// If you find this work to be useful
// please consider refering to it. 

#ifndef USER_INTERPRETER_H
#define USER_INTERPRETER_H

// Standard import
#include <string>
#include <vector>

// Messages
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>

// MoveIt and ROS
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "ros/ros.h"

class UserInterpreter{
    std::vector<geometry_msgs::PoseStamped> userTarget;

    int analyze();

};

#endif 