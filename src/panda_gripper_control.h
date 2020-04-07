#ifndef PANDA_GRIPPER_CONTROL_H
#define PANDA_GRIPPER_CONTROL_H

#include "panda_gripper_control.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

void VR_triggerCallback(const std_msgs::Float32::ConstPtr& msg);
void VR_gripCallback(const std_msgs::Int32::ConstPtr& msg);


#endif //PANDA_GRIPPER_CONTROL_H