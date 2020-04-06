#ifndef PANDA_DUAL_CONTROL_H
#define PANDA_DUAL_CONTROL_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "panda_dual_control.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

void triggerCallback(const std_msgs::Float32::ConstPtr& msg);
void controllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void controllerCallbackProcessed(const geometry_msgs::PoseStamped::ConstPtr& msg);
void SubtractPose(geometry_msgs::Pose &pose, geometry_msgs::Pose startingPose);
void AddPose(geometry_msgs::Pose &pose, geometry_msgs::Pose originPose);
void menuCallback(const std_msgs::Int32::ConstPtr& msg);

class RobotHandler {
    public:
        moveit::planning_interface::MoveGroupInterface *move_group_p;
        // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // RobotHandler(moveit::planning_interface::MoveGroupInterface *move_group_p);
        // void initPosition();

};


class CallibrationManager {
    public:
        geometry_msgs::Pose startingPose;
        geometry_msgs::Pose originPose;
        void SetStartPose(geometry_msgs::Pose msg);
        bool setStartPose = false;
                
};
// RobotHandler::RobotHandler(moveit::planning_interface::MoveGroupInterface *move_group_p)
// {
//     // static const std::string PLANNING_GROUP = "panda_arm";
//     // moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//     // this->move_group_p = &move_group;

//     // const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//     // namespace rvt = rviz_visual_tools;
//     // moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
//     // visual_tools.deleteAllMarkers();
// }

#endif //PANDA_DUAL_CONTROL_H