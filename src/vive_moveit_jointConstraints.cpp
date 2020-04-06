#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "vive_moveit_jointConstraints.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <cmath>

#include <tf2_ros/transform_listener.h>
// #include "LinearMath/btMatrix3x3.h"

#define _USE_MATH_DEFINES



RobotHandler robotHandler;
CallibrationManager callibrationManager;
ros::Publisher pub;

tf2_ros::Buffer tfBuffer;
// tf2_ros::TransformListener tfListener(tfBuffer);


double deg2rad (double degrees) {
    return degrees * 4.0 * atan (1.0) / 180.0;
}

struct Quaternion
{
    double w, x, y, z;
};

Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

Quaternion ToQuaternionFromMsg(geometry_msgs::Pose pose)
{
    Quaternion q;
    q.x = pose.orientation.x;
    q.y = pose.orientation.y;
    q.z = pose.orientation.z;
    q.w = pose.orientation.w;

    return q;
}

geometry_msgs::PoseStamped copyPose(geometry_msgs::PoseStamped poseStamped)
{
    return poseStamped;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "moveit_vive_jointConstraints");
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda2";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();

    robotHandler.move_group_p = &move_group;


    // callibrationManager.originPose = move_group.getCurrentPose().pose;
    geometry_msgs::Pose target_pose1;
    // target_pose1.orientation.w = 0.0;
    // target_pose1.orientation.x = 1.0;
    // target_pose1.orientation.y = 0.0;
    // target_pose1.orientation.z = 0.0;

    Quaternion q = ToQuaternion(0.0, deg2rad(180.0), deg2rad(0.0));
    target_pose1.orientation.w = q.w;
    target_pose1.orientation.x = q.x;
    target_pose1.orientation.y = q.y;
    target_pose1.orientation.z = q.z;
    

    target_pose1 = move_group.getCurrentPose().pose;
    // target_pose1.position.x = 0.5;
    // target_pose1.position.y = 0.0;
    target_pose1.position.z += 0.2;
    move_group.setPoseTarget(target_pose1);
    callibrationManager.originPose = target_pose1;


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("jointConstraints", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

    /* Uncomment below line when working with a real robot */
    move_group.move();
    // move_group.execute(my_plan);


    ros::Subscriber sub_processed = node_handle.subscribe("/vive/controller/right/processed/pose", 1, controllerCallbackProcessed);
    ros::Subscriber sub = node_handle.subscribe("/vive/controller/right/pose", 1, controllerCallback);
    ros::Subscriber sub_trigger = node_handle.subscribe("/vive/controller/right/trigger", 1, triggerCallback);
    pub = node_handle.advertise<geometry_msgs::PoseStamped>("/vive/controller/right/processed/pose", 1);


    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep(); // Don't forget this! *
    }

    return 0;
}

void controllerCallbackProcessed(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    robotHandler.move_group_p->setPoseTarget(msg->pose);
    robotHandler.move_group_p->move();
}

void controllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // ROS_INFO("\nGot:\n x: [%f]\n y: [%f]\n z: [%f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    geometry_msgs::PoseStamped cmsg = copyPose(*msg);

    // robotHandler.move_group_p->setPoseTarget(msg->pose);


    tf2::Quaternion q_controller, q_rot, q_processed, q_end, q_base, q_r, q_callibrationOrientation, q_startingOrientation;
    tf2::Quaternion q_callibrationRotation;
    // Get the original orientation of 'commanded_pose'
    tf2::convert(cmsg.pose.orientation , q_controller);
    tf2::convert(callibrationManager.startingPose.orientation , q_callibrationOrientation);

    double r=0, p=M_PI, y=0;
    // double r=0, p=-M_PI/2, y=M_PI/2;  // Rotate the previous pose by 180* about X
    q_startingOrientation.setRPY(r,p,y);
    q_startingOrientation.normalize();


    r=0, p=M_PI, y=0;
    q_rot.setRPY(r, p, y);

    q_processed = q_rot*q_controller;  // Calculate the new orientation
    q_processed.normalize();

    q_callibrationRotation = q_startingOrientation *  q_controller.inverse();

    if(callibrationManager.setStartPose){
        callibrationManager.startingPose = cmsg.pose;
        q_callibrationRotation = q_startingOrientation *  q_controller.inverse();
        tf2::convert(q_callibrationRotation, callibrationManager.startingPose.orientation);
    }
    SubtractPose(cmsg.pose, callibrationManager.startingPose);
    AddPose(cmsg.pose, callibrationManager.originPose);

    tf2::convert(callibrationManager.startingPose.orientation, q_callibrationRotation);

    q_processed = q_callibrationRotation * q_controller;
    q_processed.normalize();

    // Stuff the new rotation back into the pose. This requires conversion into a msg type
    tf2::convert(q_controller, cmsg.pose.orientation);
    
    // double roll, pitch, yaw;
    // geometry_msgs::TransformStamped transformStamped;
    // try{
    //     transformStamped = tfBuffer.lookupTransform("panda_link0", "controller/right", ros::Time(0)); // Get transform between panda_link0 and controller/right
    //     tf2::convert(transformStamped.transform.rotation, q_r);

    //     q_end = q_controller * q_r.inverse();
    //     tf2::Matrix3x3(q_r).getRPY(roll, pitch, yaw);
    //     printf("roll %f, pitch: %f, yaw: %f \n", roll, pitch, yaw);
    //     q_end.normalize();        
    // }
    // catch (tf2::TransformException &ex) {
    //     ROS_WARN("%s",ex.what());
    //     printf("error");
    //     ros::Duration(1.0).sleep();
    //     // continue;
    // }

    // Stuff the new rotation back into the pose. This requires conversion into a msg type
    // tf2::convert(q_new, dmsg.pose.orientation);
    // tf2::convert(q_processed, dmsg.pose.orientation);
    
    
    pub.publish(cmsg);
}

void triggerCallback(const std_msgs::Float32::ConstPtr& msg)
{
    callibrationManager.setStartPose = (msg->data == 1);
}


void CallibrationManager::SetStartPose(geometry_msgs::Pose msg){
    this->startingPose = msg;
}

void SubtractPose(geometry_msgs::Pose &pose, geometry_msgs::Pose startingPose)
{
    pose.position.x -= startingPose.position.x;
    pose.position.y -= startingPose.position.y;
    pose.position.z -= startingPose.position.z;
}

void AddPose(geometry_msgs::Pose &pose, geometry_msgs::Pose originPose)
{
    pose.position.x += originPose.position.x;
    pose.position.y += originPose.position.y;
    pose.position.z += originPose.position.z;
}