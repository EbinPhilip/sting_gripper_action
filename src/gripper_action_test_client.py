#! /usr/bin/env python

import rospy
import actionlib

from control_msgs.msg import GripperCommandAction as GCA
from control_msgs.msg import GripperCommandGoal as GCGoal
from control_msgs.msg import GripperCommand as GC

def feedback_cb(feedback):
    print("feedback")
    print("effort:"+str(feedback.effort))
    print("position:"+str(feedback.position))
    print("stalled:"+str(feedback.stalled))
    print("reached:"+str(feedback.reached_goal))
    print("")
    

def gripper_action_client():
    client = actionlib.SimpleActionClient('sting_gripper_action', GCA)

    client.wait_for_server()

    goal = GCGoal()
    goal.command.position = 0.08
    goal.command.max_effort = 0.270

    client.send_goal(goal, feedback_cb=feedback_cb)

    client.wait_for_result()

    result = client.get_result()

    print("result")
    print("effort:"+str(result.effort))
    print("position:"+str(result.position))
    print("stalled:"+str(result.stalled))
    print("reached:"+str(result.reached_goal))
    print("") 

    return result


if __name__ == '__main__':
    try:
        rospy.init_node('sting_gripper_action_client')
        result = gripper_action_client()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")