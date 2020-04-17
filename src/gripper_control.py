#!/usr/bin/env python
"""
A simple script that translates desired gripper width to command for
JointGroupPositionController.
"""
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64, Float32, Float32MultiArray

width = 0.0
arm_id = "1"

def callback(data):
    print("Setting gripper width to:", data.data)
    pub = rospy.Publisher("/panda_"+arm_id+"_hand_controller/command",
                          Float64MultiArray, queue_size=1)

    msg = Float64MultiArray()
    msg.layout.dim = [MultiArrayDimension('', 2, 1)]
    msg.data = [data.data / 2, data.data / 2]

    pub.publish(msg)


def controllerCallback(data):
    global width, arm_id
    # print("Got controller data:", data.data)
    pub = rospy.Publisher("/panda_"+arm_id+"_hand_controller/width", Float64, queue_size=1)
    if data.data[0] > 0.2:
        width += 0.005
    elif data.data[0] < -0.1:
        width -= 0.005
    
    # Block when reaching end points
    if width < 0:
        width = 0.0
    if width > 0.2:
        width = 0.2
    # msg = Float64()
    # msg.data = width
    pub.publish(width)


if __name__ == '__main__':
    rospy.init_node('gripper_publisher', anonymous=True)
    controller = rospy.get_param("~controller_id", "right")
    print(controller)
    if controller == "right":
        arm_id = "1"
    else:
        arm_id = "2"


    rospy.Subscriber("/panda_"+arm_id+"_hand_controller/width", Float64, callback, queue_size=1)
    rospy.Subscriber("/vive/controller/"+controller+"/touchpad", Float32MultiArray, controllerCallback, queue_size=1)
    # pub2 = rospy.Publisher("/panda_1_hand_controller/width", Float64, queue_size=1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    # rate = rospy.Rate(100)
    # while not rospy.is_shutdown():
        # if width < 0.1:
        #     width += 0.0001
        # else:
        #     width = 0.0
        
        # print("loop:", width)

        # pub2.publish(width)
        # rate.sleep()

