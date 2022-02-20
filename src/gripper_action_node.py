#! /usr/bin/env python

import rospy
from sting_gripper_action_server import StingGripperActionServer

if __name__ == "__main__":
    rospy.init_node('gripper_action_node')

    action_server = StingGripperActionServer()
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        action_server.executeAction()
        r.sleep()