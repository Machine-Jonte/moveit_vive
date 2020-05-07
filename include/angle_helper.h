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


double euclideanDistancePose(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
    return sqrt(
        pow(pose1.position.x - pose2.position.x, 2) +
        pow(pose1.position.y - pose2.position.y, 2) +
        pow(pose1.position.z - pose2.position.z, 2)
    );
}

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
    double sum(){
        return roll + pitch + yaw;
    }
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
