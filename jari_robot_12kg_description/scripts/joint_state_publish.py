#!/usr/bin/env python

import rospy
import random
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JariRobotVisual(object):
    def __init__(self, pub_rate, joints_name):
        self.pub_rate = pub_rate
        self.joints_name = joints_name
        self.i = -1800

    def joints_position(self):
        self.i += 1
        if self.i>1800:
            self.i=-1800
        joint1_position = self.i/1800.0*math.pi
        return [joint1_position, 0, 0, 0, 0, 0]

    def joints_state_publish(self):
        pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rospy.init_node('joint_state_publisher', anonymous=False)
        rate = rospy.Rate(self.pub_rate)
        joints_state = JointState()
        joints_state.header = Header()
        joints_state.header.stamp = rospy.Time.now()
        joints_state.name = self.joints_name 
        joints_state.velocity = []
        joints_state.effort = []
        while not rospy.is_shutdown():
            joints_state.header.stamp = rospy.Time.now()
            joints_state.position = self.joints_position()
            pub.publish(joints_state)
            rate.sleep()


if __name__ == '__main__':
    try:
        jari_visual = JariRobotVisual(500, ['base_joint', 'base_arm1_joint', 'arm1_elbow_joint', 'elbow_arm2_joint', 'arm2_wrist_joint', 'wrist_tool_joint'])
        jari_visual.joints_state_publish()
    except rospy.ROSInterruptException:
        pass

