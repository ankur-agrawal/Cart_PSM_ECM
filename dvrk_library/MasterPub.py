#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import time

rospy.init_node('LaprtotekPublisher_trial', anonymous=True)
pub_left = rospy.Publisher('/Laprotek/Lefthandle/Pose', PoseStamped, queue_size=10)

previous=0
while not rospy.is_shutdown():
    msg=PoseStamped()
    msg.header.frame_id='world'
    msg.header.stamp=rospy.Time.now()
    msg.pose.position.x=0
    msg.pose.position.y=0.001+previous
    msg.pose.position.z=0
    msg.pose.orientation.w=1
    msg.pose.orientation.x=0
    msg.pose.orientation.y=0
    msg.pose.orientation.z=0
    pub_left.publish(msg)
    # rospy.sleep(0.003)
    # print "publishing"
    # rospy.Rate(1000)
    time.sleep(0.01)
    previous = msg.pose.position.y

# raw_input("press enter to exit")
