#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "panda_dual_simultaneous_control_trajectory.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <cmath>

#include <tf2_ros/transform_listener.h>
#include "tf/LinearMath/Transform.h"
#include "tf/transform_datatypes.h"
#include <string>
#include <vector>

#include <time.h>
#include <chrono>
// #include <Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
// #include "LinearMath/btMatrix3x3.h"

#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/PositionConstraint.h>

#include "moveit_workspace.h"

#include "angle_helper.h"


#define _USE_MATH_DEFINES



RobotHandler robotHandler;
ros::Publisher pub_left;
ros::Publisher pub_right;
// ros::Publisher pub_right_controller;
// ros::Publisher pub_left_controller;
// geometry_msgs::Pose currentPose;
bool online_tracking;
bool trajectory = true;

PublishHandlerRobotState robotPublisher;

tf2_ros::Buffer tfBuffer;
// clock_t action_clock;

std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
// std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();


int main(int argc, char *argv[]) 
{
    // Initialize ROS
    ros::init(argc, argv, "panda_dual_control_moveit_vive");
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // Read param data
    std::string arm_id;
    std::string online_tracking_str;
    node_handle.param<std::string>("arm_id", arm_id, "dual");
    node_handle.param<std::string>("online_tracking", online_tracking_str, "1");
    online_tracking = std::stoi(online_tracking_str);

    // Setup MoveIt!
    static const std::string PLANNING_GROUP = arm_id;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface move_group_right("panda1");
    moveit::planning_interface::MoveGroupInterface move_group_left("panda2");

    // Addition initialization
    move_group.setPlanningTime(0.5);
    move_group_right.setPlanningTime(0.5);
    move_group_left.setPlanningTime(0.5);
    
    // move_group.setGoalOrientationTolerance(0.01);
    // move_group_right.setGoalOrientationTolerance(0.01);
    // move_group_left.setGoalOrientationTolerance(0.01);

    // move_group.setGoalPositionTolerance(0.01);
    // move_group_right.setGoalPositionTolerance(0.01);
    // move_group_left.setGoalPositionTolerance(0.01);


    

    robotHandler.right.endLinkName = "panda_1_link8";
    robotHandler.left.endLinkName = "panda_2_link8";

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    robotHandler.move_group_p = &move_group; // This is to easier reach move_group in multiple places in the code
    robotHandler.right.move_group_p = &move_group_right;
    robotHandler.left.move_group_p = &move_group_left;

    MoveItWorkSpace workspaceLeft;
    MoveItWorkSpace workspaceRight;
    robotHandler.right.workspace_p = &workspaceRight;
    robotHandler.left.workspace_p = &workspaceLeft;

    // move_group.getCurrentJointValues();
    
    // -- Setup ROS subscribers and publishers --
    // LEFT
    // ros::Subscriber sub_processed_left = node_handle.subscribe("/vive/controller/left/processed/pose", 1, controllerCallbackProcessed);
    ros::Subscriber sub_pose_left = node_handle.subscribe("/vive/controller/left/pose", 1, leftControllerCallback);
    ros::Subscriber sub_trigger_left = node_handle.subscribe("/vive/controller/left/trigger", 1, leftTriggerCallback);
    ros::Subscriber sub_menu_left = node_handle.subscribe("/vive/controller/left/buttons/menu", 1, leftMenuCallback);
    ros::Subscriber sub_grip_left = node_handle.subscribe("/vive/controller/left/buttons/grip", 1, leftGripCallback);
    pub_left = node_handle.advertise<geometry_msgs::PoseStamped>("/vive/controller/left/processed/pose", 1);
    robotHandler.left.controller_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("/panda_2_arm_controller/command", 10);


    // RIGHT
    // ros::Subscriber sub_processed_right = node_handle.subscribe("/vive/controller/right/processed/pose", 1, controllerCallbackProcessed);
    ros::Subscriber sub_pose_right = node_handle.subscribe("/vive/controller/right/pose", 1, rightControllerCallback);
    ros::Subscriber sub_trigger_right = node_handle.subscribe("/vive/controller/right/trigger", 1, rightTriggerCallback);
    ros::Subscriber sub_menu_right = node_handle.subscribe("/vive/controller/right/buttons/menu", 1, rightMenuCallback);
    ros::Subscriber sub_grip_right = node_handle.subscribe("/vive/controller/right/buttons/grip", 1, rightGripCallback);
    pub_right = node_handle.advertise<geometry_msgs::PoseStamped>("/vive/controller/right/processed/pose", 1);
    robotHandler.right.controller_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("/panda_1_arm_controller/command", 10);
    // -- --

    // robotPublisher.currentState = node_handle.advertise<geometry_msgs::PoseStamped>("/robot/currentState", 1);
    robotPublisher.jointValues = node_handle.advertise<std_msgs::Float32MultiArray>("/robot/jointValues", 1);
    robotPublisher.targetPose = node_handle.advertise<geometry_msgs::PoseStamped>("/robot/targetPose", 1);

    robotHandler.left.currentPose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("/robot/left/currentState", 1);
    robotHandler.right.currentPose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("/robot/right/currentState", 1);

    robotHandler.left.targetPose_pub = node_handle.advertise<geometry_msgs::PoseArray>("/robot/left/targetPose", 1);
    robotHandler.right.targetPose_pub = node_handle.advertise<geometry_msgs::PoseArray>("/robot/right/targetPose", 1);

    // robotHandler.left.targetPose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("/robot/left/targetPose", 1);
    // robotHandler.right.targetPose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("/robot/right/targetPose", 1);


    robotHandler.right.visualization_pub = node_handle.advertise<visualization_msgs::Marker>("robot/right/visualization_marker", 1);
    robotHandler.left.visualization_pub = node_handle.advertise<visualization_msgs::Marker>("robot/left/visualization_marker", 1);

    //Start running loop
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void moveRobot(RobotArm &robotArm)
{
    // Using IKFast then it appears that using setPose works better than calculating the cartesian path
    if(ros::Time::now().toSec() > robotArm.executionTimeEnd && trajectory){
        robotArm.waypoints = robotArm.waypointsWaiting;
        robotArm.waypointsWaiting.clear();

        double success = robotArm.PlanJointTrajectory();
        std::cout << ":::I'M TRYING TO MOVE THE ROBOT:::\nSuccess:" << success << std::endl;
        trajectory_msgs::JointTrajectory trajectory = robotArm.trajectory.joint_trajectory;
        
        robotArm.executionTimeEnd = trajectory.points.back().time_from_start.toSec() + ros::Time::now().toSec();

        geometry_msgs::PoseArray targetPose_msg = geometry_msgs::PoseArray();
        targetPose_msg.header.frame_id = "/world";
        targetPose_msg.header.stamp = ros::Time::now();
        targetPose_msg.poses = robotArm.waypoints;
        robotArm.targetPose_pub.publish(targetPose_msg);

        trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);
        // robotArm.controller_pub.publish(trajectory);
        robotHandler.move_group_p->setPoseTarget(robotArm.waypoints.back(), robotArm.endLinkName);
        robotHandler.move_group_p->asyncMove();

        // robotArm.workspace_p->remove();
    }
    else if(!trajectory) {
        robotHandler.move_group_p->setPoseTarget(robotArm.targetPose.pose, robotArm.endLinkName);
        if(robotArm.lastTimeCommand  + ros::Duration(2.0).toSec() < ros::Time::now().toSec()){
            robotArm.lastTimeCommand = ros::Time::now().toSec(); 
            
        } 
    }
    else{
        // If you want the robot to follow all your movements, uncomment this
        // However, it becomes much slower, and more awkward to control.
        robotArm.waypointsWaiting.clear();
    }
}

void ControlProcessPose(RobotArm &robotArm)
{
    if(!robotArm.menu)
    {
        robotArm.VR_startingPose = robotArm.VR_rawPose.pose;
        robotArm.basePose = robotArm.currentPoseRobot;
    }
    geometry_msgs::PoseStamped poseStamped = copyPose(robotArm.VR_rawPose);
    SubtractPose(poseStamped.pose, robotArm.VR_startingPose);
    if(robotArm.waypoints.empty() || online_tracking)
    {
        AddPose(poseStamped.pose, robotArm.basePose.pose);
    } else {
        AddPose(poseStamped.pose, robotArm.waypoints.back());
    }
    robotArm.targetPose = poseStamped;
}

void onlineUpdate(RobotArm &robotArm){
    if(robotArm.menu){
        tf2::Quaternion q_target, q_current;// q_rel;
        tf2::convert(robotArm.targetPose.pose.orientation, q_target);
        tf2::convert(robotArm.currentPoseRobot.pose.orientation, q_current);

        // q_rel = q_target * q_current.inverse();
        // q_rel.normalize();

        // EulerAngles angles = ToEulerAngles(q_rel);
        tf2Scalar angle = q_target.angleShortestPath(q_current);
        std::cout << "Difference in angle::::" << angle << std::endl;


        robotArm.finalTargetPose = robotArm.targetPose;
        // if(!robotArm.waypoints.empty())
        // {
        //     double distance = abs( euclideanDistancePose(robotArm.waypoints.back(),
        //             robotArm.currentPoseRobot.pose) );
                
        //     std::cout << "Distance:::::" << distance << std::endl;
        // }
        
        double error_distance = 0.03;
        // Only move if target is sufficiently different from current pose
        if(euclideanDistancePose(robotArm.currentPoseRobot.pose, robotArm.targetPose.pose) > error_distance || angle > 0.1 || !trajectory)
        {
            robotArm.waypointsWaiting.push_back(robotArm.targetPose.pose);
            moveRobot(robotArm);
        }
    }
}

void DualControlProcessPose(){
    if(!robotHandler.busy)
    {
        robotHandler.busy = true; // Unsure if needed

        robotHandler.left.currentPoseRobot = robotHandler.move_group_p->getCurrentPose(robotHandler.left.endLinkName);
        robotHandler.right.currentPoseRobot = robotHandler.move_group_p->getCurrentPose(robotHandler.right.endLinkName);
        std::vector<double> jointValues = robotHandler.move_group_p->getCurrentJointValues();
        // robotHandler.left.jointValues = robotHandler.move_group_p->getCurrentJointValues();
        ControlProcessPose(robotHandler.left);
        ControlProcessPose(robotHandler.right);
            
        pub_left.publish(robotHandler.left.targetPose);
        pub_right.publish(robotHandler.right.targetPose);

        // Publish data so that it can be used else where
        robotHandler.right.currentPose_pub.publish(robotHandler.right.currentPoseRobot);
        robotHandler.left.currentPose_pub.publish(robotHandler.left.currentPoseRobot);


        robotPublisher.jointValues.publish(jointValues);
        robotPublisher.currentState.publish(robotHandler.left.currentPoseRobot);
        robotPublisher.currentState.publish(robotHandler.right.currentPoseRobot);

        PublishVisualization(robotHandler.left);
        PublishVisualization(robotHandler.right);
        
        if(robotHandler.lastMoveTime + ros::Duration(2.0).toSec() < ros::Time::now().toSec()
            && !trajectory && online_tracking
            && (robotHandler.right.menu || robotHandler.left.menu)){
            robotHandler.lastMoveTime = ros::Time::now().toSec();
            robotHandler.move_group_p->asyncMove();
            // robotHandler.move_group_p->clearPoseTargets();
            
        }

        robotHandler.busy = false;// Unsure if needed
    }
}

// Process the pose from the VR to match robot (without this the pose would not be centered around the end effector)
void rightControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    robotHandler.right.VR_rawPose = *msg;
    DualControlProcessPose();
    if(online_tracking){
        onlineUpdate(robotHandler.right);
    }
}

void leftControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    robotHandler.left.VR_rawPose = *msg;
    DualControlProcessPose();
    if(online_tracking){
        onlineUpdate(robotHandler.left);
    }
}

void addTarget(RobotArm &robotArm)
{
    if(robotArm.waypoints.empty())
    {
        robotArm.waypoints.push_back(robotArm.currentPoseRobot.pose);
        robotArm.waypoints.push_back(robotArm.currentPoseRobot.pose);
    }
    robotArm.finalTargetPose = robotArm.targetPose;
    // robotArmgroup_p->setPoseTarget(robotArm.finalTargetPose.pose, robotArm.endLinkName);
    robotArm.waypoints.push_back(robotArm.finalTargetPose.pose);
    robotArm.VR_startingPose = robotArm.VR_rawPose.pose;
    robotArm.waypoints_draw = robotArm.waypoints;
}
// Add controller pose to waypoints
void rightTriggerCallback(const std_msgs::Float32::ConstPtr& msg)
{
    robotHandler.right.trigger = msg->data;
    if((int) msg->data == 1)
    {
        addTarget(robotHandler.right);
    }
}

void leftTriggerCallback(const std_msgs::Float32::ConstPtr& msg)
{
    robotHandler.left.trigger = msg->data;
    if((int) msg->data == 1)
    {
        addTarget(robotHandler.left);
    }

}

// Set the state of the menu button
// Note, if menu == 1 => controller pose can move relative to robot
void rightMenuCallback(const std_msgs::Int32::ConstPtr& msg)
{
    robotHandler.right.menu = msg->data;
}
void leftMenuCallback(const std_msgs::Int32::ConstPtr& msg)
{
    robotHandler.left.menu = msg->data;
}

// -- Move the robot according to the saved poses in waypoints --
void rightGripCallback(const std_msgs::Int32::ConstPtr& msg)
{
    robotHandler.right.grip = msg->data;
    if(msg->data == 1 && robotHandler.right.waypoints.size() > 2)
    {
        moveRobot(robotHandler.right);
    }
}

void leftGripCallback(const std_msgs::Int32::ConstPtr& msg)
{
    robotHandler.left.grip = msg->data;
    if(msg->data == 1 && robotHandler.left.waypoints.size() > 2)
    {
        moveRobot(robotHandler.left);
    }
}
// -- --


void PublishVisualization(RobotArm robotArm)
{
    visualization_msgs::Marker points, line_strip;

    points.header.frame_id = line_strip.header.frame_id = "/world";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;


    points.id = 0;
    line_strip.id = 1;
 
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.05;
    points.scale.y = 0.05;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.01;
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;
    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    for(int i = 0; i < robotArm.waypoints_draw.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = robotArm.waypoints_draw[i].position.x;
        p.y = robotArm.waypoints_draw[i].position.y;
        p.z = robotArm.waypoints_draw[i].position.z;

        points.points.push_back(p);
        line_strip.points.push_back(p);
    }
    robotArm.visualization_pub.publish(points);
    robotArm.visualization_pub.publish(line_strip);

    // visualization_pub.publish
}

geometry_msgs::PoseStamped copyPose(geometry_msgs::PoseStamped poseStamped)
{
    return poseStamped;
}

// Help functions
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

double euclideanDistancePose(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
    return sqrt(
        pow(pose1.position.x - pose2.position.x, 2) +
        pow(pose1.position.y - pose2.position.y, 2) +
        pow(pose1.position.z - pose2.position.z, 2)
    );
}

moveit_msgs::BoundingVolume createBoundingVolume(std::vector<double> size, geometry_msgs::Pose boundingBoxPose)
{
    moveit_msgs::BoundingVolume boundingVolume;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = size[0]; // x
    primitive.dimensions[1] = size[1]; // y
    primitive.dimensions[2] = size[2]; // z

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = boundingBoxPose.position.x;
    box_pose.position.y = boundingBoxPose.position.y;
    box_pose.position.z = boundingBoxPose.position.z;

    boundingVolume.primitives.push_back(primitive);
    boundingVolume.primitive_poses.push_back(box_pose);
    // boundingVolume.operation = boundingVolume.ADD;


    return boundingVolume;
}
