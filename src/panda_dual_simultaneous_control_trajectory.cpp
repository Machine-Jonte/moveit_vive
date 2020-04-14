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
#include <string>
#include <vector>
#include <time.h>
// #include "LinearMath/btMatrix3x3.h"

#define _USE_MATH_DEFINES



RobotHandler robotHandler;
ros::Publisher pub_left;
ros::Publisher pub_right;
ros::Publisher pub_right_controller;
ros::Publisher pub_left_controller;
// geometry_msgs::Pose currentPose;

tf2_ros::Buffer tfBuffer;
// clock_t action_clock;


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
    node_handle.param<std::string>("arm_id", arm_id, "dual");

    // Setup MoveIt!
    static const std::string PLANNING_GROUP = arm_id;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface move_group_right("panda1");
    moveit::planning_interface::MoveGroupInterface move_group_left("panda2");
    // move_group.setEndEffectorLink("panda_1_link8");
    // move_group.setEndEffectorLink("panda_2_link8");

    robotHandler.right.endLinkName = "panda_1_link8";
    robotHandler.left.endLinkName = "panda_2_link8";

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    robotHandler.move_group_p = &move_group; // This is to easier reach move_group in multiple places in the code

    
    // Setup ROS subscribers and publishers
    // LEFT
    // ros::Subscriber sub_processed_left = node_handle.subscribe("/vive/controller/left/processed/pose", 1, controllerCallbackProcessed);
    ros::Subscriber sub_pose_left = node_handle.subscribe("/vive/controller/left/pose", 1, leftControllerCallback);
    ros::Subscriber sub_trigger_left = node_handle.subscribe("/vive/controller/left/trigger", 1, leftTriggerCallback);
    ros::Subscriber sub_menu_left = node_handle.subscribe("/vive/controller/left/buttons/menu", 1, leftMenuCallback);
    ros::Subscriber sub_grip_left = node_handle.subscribe("/vive/controller/left/buttons/grip", 1, leftGripCallback);
    pub_left = node_handle.advertise<geometry_msgs::PoseStamped>("/vive/controller/left/processed/pose", 1);
    pub_left_controller = node_handle.advertise<trajectory_msgs::JointTrajectory>("/panda_2_arm_controller/command", 10);


    // RIGHT
    // ros::Subscriber sub_processed_right = node_handle.subscribe("/vive/controller/right/processed/pose", 1, controllerCallbackProcessed);
    ros::Subscriber sub_pose_right = node_handle.subscribe("/vive/controller/right/pose", 1, rightControllerCallback);
    ros::Subscriber sub_trigger_right = node_handle.subscribe("/vive/controller/right/trigger", 1, rightTriggerCallback);
    ros::Subscriber sub_menu_right = node_handle.subscribe("/vive/controller/right/buttons/menu", 1, rightMenuCallback);
    ros::Subscriber sub_grip_right = node_handle.subscribe("/vive/controller/right/buttons/grip", 1, rightGripCallback);
    pub_right = node_handle.advertise<geometry_msgs::PoseStamped>("/vive/controller/right/processed/pose", 1);
    pub_right_controller = node_handle.advertise<trajectory_msgs::JointTrajectory>("/panda_1_arm_controller/command", 10);

    //Start running loop
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ros::AsyncSpinner spinner(1); 
        spinner.start();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

// Read the processed pose and IF button is pressed move

// Set target pose and move robot
void moveRobot()
{
    // if(robotHandler.left.menu)
    // {
    //     // robotHandler.left.PlanAndExecuteTrajectory(robotHandler.move_group_p);
    //     // robotHandler.right.PlanAndExecuteTrajectory(robotHandler.move_group_p);
        // robotHandler.move_group_p->setPoseTarget(robotHandler.left.finalTargetPose.pose, robotHandler.left.endLinkName);
    // }
    // if(robotHandler.right.menu)
    // {
    //     // robotHandler.left.PlanAndExecuteTrajectory(robotHandler.move_group_p);
    //     robotHandler.move_group_p->setPoseTarget(robotHandler.right.finalTargetPose.pose, robotHandler.right.endLinkName);
    // }

    // if(robotHandler.left.menu || robotHandler.right.menu){
    //     robotHandler.move_group_p->asyncMove();
    // }
    robotHandler.move_group_p->asyncMove();

}

void ControlProcessPose(RobotArm &robotArm)
{
    if(!robotArm.menu)
    {
        robotArm.VR_startingPose = robotArm.VR_rawPose.pose;
    }
    geometry_msgs::PoseStamped poseStamped = copyPose(robotArm.VR_rawPose);
    SubtractPose(poseStamped.pose, robotArm.VR_startingPose);
    AddPose(poseStamped.pose, robotArm.currentPoseRobot);
    robotArm.targetPose = poseStamped;
}

void DualControlProcessPose(){
    if(!robotHandler.busy)
    {
        robotHandler.busy = true; // Unsure if needed

        robotHandler.left.currentPoseRobot = robotHandler.move_group_p->getCurrentPose(robotHandler.left.endLinkName).pose;
        robotHandler.right.currentPoseRobot = robotHandler.move_group_p->getCurrentPose(robotHandler.right.endLinkName).pose;
        ControlProcessPose(robotHandler.left);
        ControlProcessPose(robotHandler.right);
            
        pub_left.publish(robotHandler.left.targetPose);
        pub_right.publish(robotHandler.right.targetPose);

        // moveRobot();

        robotHandler.busy = false;// Unsure if needed
    }
}


// geometry_msgs::Pose ControllerProcessPose(geometry_msgs::Pose pose, RobotArm *robotArm)
// {
//     if(!(robotArm->menu == 1)){
//         printf("Got here\n");
//         robotArm->VR_startingPose = pose;
//         try{
//             printf("Got here too!\n");
//             std::cout << robotArm->endLinkName << std::endl;
//             robotArm->currentPoseRobot = robotHandler.move_group_p->getCurrentPose(robotArm->endLinkName).pose;
//             // printf("Current Pose: x: %f, y: %f, z: %f \n", currentPose.position.x, currentPose.position.y, currentPose.position.z);
//         }
//         catch (int e) {
//             printf("Couldn't get current pose\n");
//         }
//     }
//     SubtractPose(pose, robotArm->VR_startingPose);
//     AddPose(pose, robotArm->currentPoseRobot);
//     robotArm->targetPose = pose;

//     return pose;
// }
// Process the pose from the VR to match robot (without this the pose would not be centered around the end effector)
// It would be prefferable to combine both of the callbacks as they are redicously similar.
void rightControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // It's only possible to run getCurrentPose() one time at the time
    // if(!robotHandler.busy)
    // {
    //     robotHandler.busy = true;
    //     geometry_msgs::PoseStamped cmsg = copyPose(*msg);
    //     cmsg.pose = ControllerProcessPose(cmsg.pose, &robotHandler.right);
    //     pub_right.publish(cmsg);
    //     robotHandler.busy = false;
    // }
    robotHandler.right.VR_rawPose = *msg;
    DualControlProcessPose();
    // pub_right.publish(robotHandler.right.targetPose);
}

void leftControllerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // It's only possible to run getCurrentPose() one time at the time
    // if(!robotHandler.busy)
    // {
    //     robotHandler.busy = true;
    //     geometry_msgs::PoseStamped cmsg = copyPose(*msg);
    //     cmsg.pose = ControllerProcessPose(cmsg.pose, &robotHandler.left);
    //     pub_left.publish(cmsg);
    //     robotHandler.busy = false;
    // }
    robotHandler.left.VR_rawPose = *msg;
    DualControlProcessPose();
    // pub_left.publish(robotHandler.left.targetPose);
}


// Move the grippers (Open/Close)
void rightTriggerCallback(const std_msgs::Float32::ConstPtr& msg)
{
    robotHandler.right.trigger = msg->data;
    if((int) msg->data == 1)
    {
        robotHandler.right.finalTargetPose = robotHandler.right.targetPose;
        robotHandler.move_group_p->setPoseTarget(robotHandler.right.finalTargetPose.pose, robotHandler.right.endLinkName);
        
    }
}

void leftTriggerCallback(const std_msgs::Float32::ConstPtr& msg)
{
    robotHandler.left.trigger = msg->data;
    if((int) msg->data == 1)
    {
        robotHandler.left.finalTargetPose = robotHandler.left.targetPose;
        robotHandler.move_group_p->setPoseTarget(robotHandler.left.finalTargetPose.pose, robotHandler.left.endLinkName);
    }

}

// If pressed => move robot 
void rightMenuCallback(const std_msgs::Int32::ConstPtr& msg)
{
    robotHandler.right.menu = msg->data;
}
void leftMenuCallback(const std_msgs::Int32::ConstPtr& msg)
{
    robotHandler.left.menu = msg->data;
}

void rightGripCallback(const std_msgs::Int32::ConstPtr& msg)
{
    robotHandler.left.grip = msg->data;
    if(msg->data == 1)
    {
        moveRobot();
    }
}

void leftGripCallback(const std_msgs::Int32::ConstPtr& msg)
{
    robotHandler.left.grip = msg->data;
    if(msg->data == 1)
    {
        moveRobot();
    }
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



// void RobotArm::PlanAndExecuteTrajectory(moveit::planning_interface::MoveGroupInterface *move_group_p){
//     this->waypoints.clear();
//     this->waypoints.push_back(this->targetPose);

//     char charEndLinkName[this->endLinkName.length() + 1]; 
//     strcpy(charEndLinkName, this->endLinkName.c_str()); 

//     move_group_p->computeCartesianPath(this->waypoints, this->eef_step, this->jump_threshold, this->trajectory, charEndLinkName);
//     this->my_plan.trajectory_ = this->trajectory;
//     move_group_p->execute(this->my_plan);
// }

// RobotHandler::RobotHandler(){
//     left.parentRobotHandler = this;
//     right.parentRobotHandler = this;
// };
