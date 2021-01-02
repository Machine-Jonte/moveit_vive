// Writtin by Jonathan Österberg
// For questions please contact: 
//                   Jonte_ost@live.com

// If you find this work to be useful
// please consider refering to it. 

// It should be possible to use this code with any other
// robotics system which is configured in MoveIt.
// For good performance, please use IKFast as 
// kinematics solver.

//Standard
#include <string>
#include <vector>

//MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/PositionConstraint.h>

#include <assert.h>     /* assert */

// Custom built
#include "robotarm.h" // This is to control data in and out

int main(int argc, char *argv[])
{
    // Initialize ROS
    ros::init(argc, argv, "moveit_controller");
    ros::NodeHandle node_handle("~");
    
    // This is used to increase the number of
    // threads used. It needs more threads 
    // because otherwise it will fail to get
    // the current pose of the robot
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // Read param data
    std::string rightControllerName;
    std::string leftControllerName;
    std::string robotPlanningGroup;
    std::string rightEndLinkName;
    std::string leftEndLinkName;
    int numberOfArms;

    std::vector<std::string> controllerName;
    std::vector<std::string> endLinkName; 

    node_handle.param<std::string>("right_controller_name", rightControllerName, "right");
    node_handle.param<std::string>("left_controller_name", leftControllerName, "left");
    node_handle.param<std::string>("robot_planning_group", robotPlanningGroup, "dual");
    node_handle.param<std::string>("right_end_link_name", rightEndLinkName, "panda_1_link8");
    node_handle.param<std::string>("left_end_link_name", leftEndLinkName, "panda_2_link8");
    node_handle.param<int>("number_of_arms", numberOfArms, 2);
    std::cout << "[MOVEIT_VIVE]: NUMBER OF ARMS: " << numberOfArms << std::endl;
    
    assert(numberOfArms > 0);
    controllerName.push_back(rightControllerName);
    controllerName.push_back(leftControllerName);
    endLinkName.push_back(rightEndLinkName);
    endLinkName.push_back(leftEndLinkName);

    // Setup MoveIt
    moveit::planning_interface::MoveGroupInterface move_group(robotPlanningGroup);

    
    // Addition initialization
    // move_group.setPlanningTime(2.0);
    // move_group.setPlanningTime(20.0);
    // move_group.setGoalOrientationTolerance(0.01);
    // move_group.setGoalPositionTolerance(0.01);

    // Set up the robot arms (publishers and subscribers)    
    Robot robot(numberOfArms);
    for(int i = 0; i < numberOfArms; i++){
        robot.robotArms[i].init(node_handle, endLinkName[i], controllerName[i]);
    }

    robot.move_group_p = &move_group;

    double publish_frequency = 10;

    //Start running loop
    ros::Rate loop_rate(publish_frequency);
    while (ros::ok()) {
        if(std::any_of(robot.robotArms.begin(), robot.robotArms.end(), [](RobotArm arm){return arm.controllerState.trigger > 0.99;}))
        {
            if(std::any_of(robot.robotArms.begin(), robot.robotArms.end(), [](RobotArm arm) {return !arm.controllerState.grip;}))
            {
                std::cout << "[MOVEIT_VIVE]: TRYING TO MOVE" << std::endl;
                robot.setPoseTargets();
                robot.move_group_p->move();
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
