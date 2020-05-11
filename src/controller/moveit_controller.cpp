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


// Custom built
#include "robotarm.h" // This is to control data in and out
#include "user_interpreter.h"



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
    std::string dualArmPlanningGroup;
    std::string rightArmPlanningGroup;
    std::string leftArmPlanningGroup;
    std::string rightEndLinkName;
    std::string leftEndLinkName;

    node_handle.param<std::string>("right_controller_name", rightControllerName, "right");
    node_handle.param<std::string>("left_controller_name", leftControllerName, "left");
    node_handle.param<std::string>("dual_planning_group", dualArmPlanningGroup, "dual");
    node_handle.param<std::string>("right_end_link_name", rightEndLinkName, "panda_1_link8");
    node_handle.param<std::string>("left_end_link_name", leftEndLinkName, "panda_2_link8");
    


    // Setup MoveIt
    moveit::planning_interface::MoveGroupInterface move_group(dualArmPlanningGroup);

    
    // Addition initialization
    move_group.setPlanningTime(0.5);
    // move_group.setGoalOrientationTolerance(0.01);
    // move_group.setGoalPositionTolerance(0.01);

    // Set up the robot arms (publishers and subscribers)    
    Robot robot;
    robot.right.init(node_handle, rightEndLinkName, rightControllerName);
    robot.left.init(node_handle, leftEndLinkName, leftControllerName);
    robot.move_group_p = &move_group;



    // User Interpreter (to know when to send commands to move or not)
    double publish_frequency = 10;
    UserInterpreter userInterpreter;
    userInterpreter.lookbackTime = 0.2;
    userInterpreter.init(&robot); 
    userInterpreter.publish_frequency = publish_frequency;

    //Start running loop
    ros::Rate loop_rate(publish_frequency);
    while (ros::ok()) {
        userInterpreter.PushBackData();
        if(userInterpreter.Analyze())
        {
            robot.setPoseTargets();
            robot.move_group_p->move();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
