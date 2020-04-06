#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "panda_dual_control.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <cmath>

#include <tf2_ros/transform_listener.h>
#include <string>
// #include "LinearMath/btMatrix3x3.h"

#define _USE_MATH_DEFINES



RobotHandler robotHandler;
CallibrationManager callibrationManager;
ros::Publisher pub;
geometry_msgs::Pose currentPose;

tf2_ros::Buffer tfBuffer;


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

int main(int argc, char *argv[]) 
{
    // Initialize ROS
    ros::init(argc, argv, "panda_dual_control_moveit_vive");
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Read param data
    std::string arm_id;
    node_handle.param<std::string>("arm_id", arm_id, "panda2");

    std::string controller_id;
    node_handle.param<std::string>("controller_id", controller_id, "left");

    // Setup MoveIt!
    static const std::string PLANNING_GROUP = arm_id;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    robotHandler.move_group_p = &move_group; // This is to easier reach move_group in multiple places in the code

    // Setup ROS subscribers and publishers
    ros::Subscriber sub_processed = node_handle.subscribe("/vive/controller/" + controller_id + "/processed/pose", 1, controllerCallbackProcessed);
    ros::Subscriber sub = node_handle.subscribe("/vive/controller/" + controller_id + "/pose", 1, controllerCallback);
    // ros::Subscriber sub_trigger = node_handle.subscribe("/vive/controller/" + controller_id + "/trigger", 1, triggerCallback);
    ros::Subscriber sub_menu = node_handle.subscribe("/vive/controller/" + controller_id + "/buttons/menu", 1, menuCallback);
    pub = node_handle.advertise<geometry_msgs::PoseStamped>("/vive/controller/" + controller_id + "/processed/pose", 1);

    //Start running loop
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::AsyncSpinner spinner(1); 
        spinner.start();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

// Read the processed pose and IF button is pressed move
void controllerCallbackProcessed(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(callibrationManager.setStartPose)
    {
        robotHandler.move_group_p->setPoseTarget(msg->pose);
        robotHandler.move_group_p->move();
    }
}

// Process the pose from the VR to match robot (without this the pose would not be centered around the end effector)
void controllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::PoseStamped cmsg = copyPose(*msg);

    if(!callibrationManager.setStartPose){
        callibrationManager.startingPose = cmsg.pose;
        try{
            currentPose = robotHandler.move_group_p->getCurrentPose().pose;
        }
        catch (int e) {
            printf("Couldn't get current pose");
        }
    }
    SubtractPose(cmsg.pose, callibrationManager.startingPose);
    AddPose(cmsg.pose, currentPose);

    pub.publish(cmsg);
}

// Move the grippers (Open/Close)
// void triggerCallback(const std_msgs::Float32::ConstPtr& msg)
// {
//     std::vector<double> joints;
//     joints = robotHandler.move_group_p->getCurrentJointValues();
//     printf("Joint Value %f", joints.at(0));
//     joints.at(0) = msg->data;

//     robotHandler.move_group_p->setJointValueTarget(joints);
//     robotHandler.move_group_p->move();
// }

// If pressed => move robot 
void menuCallback(const std_msgs::Int32::ConstPtr& msg)
{
    callibrationManager.setStartPose = (msg->data == 1);
}

// Help functions
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
