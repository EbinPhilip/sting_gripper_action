#! /usr/bin/env python

import time
from datetime import datetime

import rospy
import actionlib

from sensor_msgs.msg import JointState
from control_msgs.msg import GripperCommandAction as GCAction
from control_msgs.msg import GripperCommandResult as GCResult
from control_msgs.msg import GripperCommandFeedback as GCFeedback
from control_msgs.msg import GripperCommand as GC
from std_msgs.msg import Float64

from math import fabs

count = 0

class StingGripperActionServer:
    def __init__(self):

        self.action_name = rospy.get_param("action_name", "sting_gripper_action")
        self.gripper_joint_name = rospy.get_param("gripper_joint_name", "sting_gripper_joint")
        self.joint_state_topic = rospy.get_param("joint_state_topic","/joint_states")
        self.gripper_command_topic = rospy.get_param("gripper_command_topic", "sting_gripper_controller/command")

        self.action = actionlib.SimpleActionServer(self.action_name, GCAction, auto_start=False)
        self.action.register_goal_callback(self.goalCB)
        self.action.register_preempt_callback(self.preemptCB)
        self.action.start()

        self.joint_pos = None
        self.joint_effort = None

        self.goal_pos = None
        self.goal_effort = None

        self.cmd_pub = rospy.Publisher(self.gripper_command_topic, Float64, queue_size=1)

        self.joint_state_timestamp = None
        rospy.Subscriber(self.joint_state_topic, JointState, self.jointStateSubscriberCB)

    def jointStateSubscriberCB(self, joint_states=JointState()):
        idx = None
        for (i,joint) in enumerate(joint_states.name):
            if (joint == self.gripper_joint_name):
                idx = i
                break
        
        if (idx == None):
            rospy.logerr("Joint: %s not found",self.gripper_joint_name)
            return

        self.joint_pos = joint_states.position[idx]
        self.joint_effort = joint_states.effort[idx]
        self.joint_state_timestamp = datetime.now()

    def goalCB(self):
        gripper_goal = self.action.accept_new_goal()
        self.goal_pos = gripper_goal.command.position/2.0
        self.goal_effort = gripper_goal.command.max_effort
        self.action_attempt_count = 0

    def preemptCB(self):
        self.action.set_preempted()

    def executeAction(self):
        if (not self.action.is_active()):
            return
        
        if (self.joint_pos==None or self.joint_effort==None or self.joint_state_timestamp==None):
            rospy.logwarn("joint states unknown!")
            self.action_attempt_count = self.action_attempt_count+1
            if(self.action_attempt_count>10):
                result = GCResult()
                result.stalled = False
                result.reached_goal = False
                err_string = "joint state unknown. action aborted!"
                self.action.set_aborted(result, err_string)
                rospy.logerr(err_string)
            return
        self.action_attempt_count = 0

        if ((datetime.now() - self.joint_state_timestamp).total_seconds() >= 1):
            result = GCResult()
            result = GCResult()
            result.stalled = True
            result.reached_goal = False
            err_string = "actuator offline. action aborted!"
            self.action.set_aborted(result, err_string)
            rospy.logerr(err_string)
            return

        if fabs(self.joint_pos-self.goal_pos)>0.001 and fabs(self.joint_effort)<=self.goal_effort:
            self.cmd_pub.publish(self.goal_pos)
            feedback = GCFeedback()
            feedback.effort = self.joint_effort
            feedback.position = self.joint_pos*2.0
            feedback.stalled = False
            feedback.reached_goal = False
            self.action.publish_feedback(feedback)
        else:
            if (self.joint_effort > self.goal_effort):
                self.cmd_pub.publish(self.joint_pos-0.0005)
                time.sleep(1)
            result = GCResult()
            result.effort = self.joint_effort
            result.position = self.joint_pos*2.0
            result.stalled = False
            result.reached_goal = True
            self.action.set_succeeded(result)