#include "panda_gripper_control.h"
#include "ros/ros.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "panda_grip_control_moveit_vive");
    ros::NodeHandle node_handle("~");

    // Read param data
    std::string arm_id;
    node_handle.param<std::string>("arm_id", arm_id, "panda2");

    std::string controller_id;
    node_handle.param<std::string>("controller_id", controller_id, "left");

    // Launch publishers and subscribers
    ros::Subscriber sub_trigger = node_handle.subscribe("/vive/controller/" + controller_id + "/trigger", 1, VR_triggerCallback);
    ros::Subscriber sub_menu = node_handle.subscribe("/vive/controller/" + controller_id + "/buttons/grip", 1, VR_gripCallback);

    return 0;
}


void VR_triggerCallback(const std_msgs::Float32::ConstPtr& msg)
{

}


void VR_gripCallback(const std_msgs::Int32::ConstPtr& msg)
{

}