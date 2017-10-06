#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('/psm_joint_2_position_controller/command',Float64, queue_size=10)
    rospy.init_node('Talkity_talk_talk', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
      hello_str =-0.8
      rospy.loginfo(hello_str)
      pub.publish(hello_str)
      rate.sleep()
	

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
