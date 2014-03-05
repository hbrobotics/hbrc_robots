#!/usr/bin/env python

import roslib; roslib.load_manifest("scout_bringup")
import rospy
from sensor_msgs.msg import JointState

rospy.init_node("fake_wheel_pub")
pub = rospy.Publisher('joint_states', JointState)

rate = 10
r = rospy.Rate(rate)

msg = JointState()
msg.name = ["base_l_wheel_joint", "base_r_wheel_joint"]
msg.position = [0.0 for name in msg.name]
msg.velocity = [0.0 for name in msg.name]

while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    r.sleep()

