#!/usr/bin/env python
# http://wiki.ros.org/rospy/Overview/Messages

#rostopic pub /dvrk/PSM1/set_robot_state std_msgs/String DVRK_POSITION_CARTESIAN
# rostopic info /dvrk/MTML/set_position_joint

import rospy
import math
import time
import csv
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

r=5 #frequency Hz of the publisher and subscriber

def callback_joint(msg):
  global joint_sub
  joint_sub=msg

def main(ra):
    # Create Pose Publisher and Subscriber
    joint_pub= rospy.Publisher('/dvrk/PSM1/set_position_joint', JointState, queue_size=10)
    rospy.Subscriber('/dvrk/PSM1/state_joint_current', JointState, callback_joint)
    
    rospy.init_node('Talkity_talk_talk',anonymous=True)
    rate = rospy.Rate(ra) # 1hz
    

    joint_msg= JointState()
    joint_msg.header = Header()
    joint_msg.header.stamp = rospy.Time.now()
    joint_msg.name = ['outer_yaw', 'outer_pitch', 'outer_roll', 'outer_insertion', 'outer__wrist_pitch','outer_wrist_jaw', 'jaw',]
    joint_msg.position = [0]*3
    joint_msg.velocity = []  
    joint_msg.effort = []
    i=0

    q_data1= [0,-0.01522,-0.10144,-0.28026,-0.53248,-0.8125,-1.0627,-1.2279,-1.2698,-1.181,-1,-0.79762,-0.58544,-0.36436,-0.14368,0.0625,0.23888,0.37256,0.45664,0.49382,0.5,0.48478,0.39856,0.21974,-0.03248,-0.3125,-0.56272,-0.72794,-0.76976,-0.68098,-0.5,-0.5,-0.14336,-0.02744,0.03888,0.0625,0.05632,0.03564,0.01456,0.00238,0]    
    q_data2= [0,0.00152,0.01152,0.03672,0.08192,0.15,0.24192,0.35672,0.49152,0.64152,0.8,0.95163,1.0621,1.1128,1.1041,1.05,0.97203,0.89374,0.83482,0.80533,0.8,0.80533,0.83482,0.89374,0.97203,1.05,1.1041,1.1128,1.0621,0.95163,0.8,0.8,0.49152,0.35672,0.24192,0.15,0.08192,0.03672,0.01152,0.00152,0 ]  
    q_data3= [0.13,0.12867,0.1213,0.10656,0.086992,0.0675,0.053968,0.051796,0.064464,0.092092,0.13,0.16735,0.19177,0.1976,0.1854,0.16,0.12864,0.099036,0.077469,0.066888,0.065,0.064334,0.060648,0.053282,0.043496,0.03375,0.026984,0.025898,0.032232,0.046046,0.065,0.065,0.094003,0.093502,0.082382,0.06375,0.042138,0.022318,0.0081168,0.0012224,0]   
    
   
    q_output=[None]*len(q_data1)
    qd_output=[None]*len(q_data1)
    torque_output=[None]*len(q_data1)

    rospy.sleep(1)
    joint_pub.publish(joint_msg)
    rospy.sleep(3)

    while not rospy.is_shutdown():
      if i>len(q_data1)-1:
        break
      #joint_msg.position = [q_data1[i], q_data2[i], q_data3[i], 0, 0, 0, 0]
      joint_msg.position[0]=q_data1[i]
      joint_msg.position[1]=q_data2[i]
      joint_msg.position[2]=q_data3[i]
      joint_msg.velocity = []  
      joint_msg.effort = []
      joint_pub.publish(joint_msg)

#      q_output[i]=joint_sub.position
#      qd_output[i]=joint_sub.velocity
#      torque_output[i]=joint_sub.effort

      rospy.sleep(1/float(ra))
      i+=1
    rate.sleep()

#    with open('data_position.csv', 'wb') as myfile:
#        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
#        for i in range(0,len(q_data1)):
#          wr.writerow(q_output[i])
#    with open('data_velocity.csv', 'wb') as myfile:
#        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
#        for i in range(0,len(q_data1)):
#         wr.writerow(qd_output[i])
#    with open('data_torque.csv', 'wb') as myfile:
#        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
#       for i in range(0,len(q_data1)):
#          wr.writerow(torque_output[i])    
    

if __name__ == '__main__':
    try:
      main(r)
      
    except rospy.ROSInterruptException:
      
      pass
