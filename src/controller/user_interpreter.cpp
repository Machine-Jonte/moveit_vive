#include "user_interpreter.h"
#include "robotarm.h"
#include "geometry_msgs/PoseStamped.h"
#include <algorithm>

void UserInterpreter::init(Robot *robot_p)
{
    this->robot_p = robot_p;
    this->left.parent = this;
    this->right.parent = this;
}

int UserInterpreter::Analyze()
{   
    // Check if any of the arms should do a move
    int result = this->right.Analyze() + this->left.Analyze();
    if(result > 0)
    {
        return 1;
    }

    return 0;
}

double euclideanDistancePose(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
    return sqrt(
        pow(pose1.position.x - pose2.position.x, 2) +
        pow(pose1.position.y - pose2.position.y, 2) +
        pow(pose1.position.z - pose2.position.z, 2)
    );
}

int DataArmHolder::Analyze()
{
    int lookBackIndex = this->parent->publish_frequency * this->parent->lookbackTime;
    std::cout << "Look back index: " << lookBackIndex << std::endl;
    std::cout << "Size: " << this->userTargets.size() << std::endl;

    if(this->userTargets.size() > lookBackIndex) {
       double controllerDeltaMovement = euclideanDistancePose( (this->userTargets.rbegin() + lookBackIndex)->pose, this->userTargets.back().pose); 
        std::cout << "Distance: " << controllerDeltaMovement << std::endl;

        // If controller hasn't moved during the given time span
        // move to next phase if it should update pose or not
        // this is so the moveit controller doesn't get overflown 
        // with commands
       if(controllerDeltaMovement < this->parent->VR_steadyError)
       {
            double controllerRobotDeviation = euclideanDistancePose(this->userTargets.back().pose, this->robotPoses.back().pose);
            std::cout << "controllerRobotDeviation: " << controllerRobotDeviation << std::endl;

            // The robot shall only move if the controller
            // isn't at the same position as the end effector
            if(controllerRobotDeviation > this->parent->robot_steadyError)
            {
                double targetError = euclideanDistancePose(this->userTargets.back().pose, this->poseMovingTo.pose);
                if(targetError > this->parent->target_error)
                {
                    this->poseMovingTo = this->userTargets.back();
                    std::cout << "Returning 1: " << std::endl;
                    return 1;
                } 
            }
       }
    }
    return 0;
}

void KeepLastElements(int nrElements, std::vector<geometry_msgs::PoseStamped> &vector)
{
    std::vector<geometry_msgs::PoseStamped> y(vector.end() - std::min((int)vector.size(), nrElements), vector.end());
    vector = y;
}

void UserInterpreter::PushBackData()
{
    this->right.robotPoses.push_back(this->robot_p->right.currentArmPose);
    this->left.robotPoses.push_back(this->robot_p->left.currentArmPose);

    this->right.userTargets.push_back(this->robot_p->right.targetPose);
    this->left.userTargets.push_back(this->robot_p->left.targetPose);


    KeepLastElements(this->lookBackMaxSizeVector, this->right.robotPoses);
    KeepLastElements(this->lookBackMaxSizeVector, this->left.robotPoses);
    KeepLastElements(this->lookBackMaxSizeVector, this->right.userTargets);
    KeepLastElements(this->lookBackMaxSizeVector, this->left.userTargets);
}
