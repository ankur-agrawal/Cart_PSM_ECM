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
    
    #hardcode home to zero 
    joint_msg= JointState()
    joint_msg.header = Header()
    joint_msg.header.stamp = rospy.Time.now()
    joint_msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer__wrist_pitch','outer_wrist_jaw', 'jaw']
    joint_msg.position=[0]*3
    joint_msg.velocity = []  
    joint_msg.effort = []
    
    rospy.sleep(1)
    d=0.04
    thresh=0.03

    while not rospy.is_shutdown():
      if -thresh<joint_sub.position[0]<thresh:
        dq1=None  
      elif joint_sub.position[0]<0: 
        dq1=joint_sub.position[0]+d
      else: 
        dq1=joint_sub.position[0]-d

      if -thresh<joint_sub.position[1]<thresh: 
        dq2=None  
      elif joint_sub.position[1]<0: 
        dq2=joint_sub.position[1]+d
      else: 
        dq2=joint_sub.position[1]-d

      if -thresh<joint_sub.position[2]<thresh:
      	dq3=None 
      elif joint_sub.position[2]<0: 
        dq3=joint_sub.position[2]+d
      else: 
        dq3=joint_sub.position[2]-d

      if -thresh<dq1<thresh and -thresh<dq2<thresh  and -thresh<dq3<thresh: 
        break

      #joint_msg.position = [dq1, dq2 , dq3 , None, None, None, None]
      joint_msg.position[0]=dq1
      joint_msg.position[1]=dq2
      joint_msg.position[2]=dq3


      joint_pub.publish(joint_msg)
      rospy.sleep(1/float(ra))

    rospy.sleep(3)
    


    #Excitation Trajectory Position Coordinates
    i=0
    q_data1=[0,-0.0014934,-0.01094,-0.033674,-0.07248,-0.12793,-0.19872,-0.28201,-0.37376,-0.46907,-0.5625,-0.64845,-0.72144,-0.7765,-0.80948,-0.81738,-0.79872,-0.75384,-0.68526,-0.59802,-0.5,-0.40198,-0.31474,-0.24616,-0.20128,-0.18262,-0.19052,-0.2235,-0.27856,-0.35155,-0.4375,-0.53093,-0.62624,-0.71799,-0.80128,-0.87207,-0.92752,-0.96633,-0.98906,-0.99851,-1,-1.0003,-1.0024,-1.0071,-1.0146,-1.0244,-1.0356,-1.0468,-1.0563,-1.0622,-1.0625,-1.0553,-1.0389,-1.0117,-0.97256,-0.9209,-0.85664,-0.78045,-0.69382,-0.59918,-0.5,-0.5,-0.31712,-0.25322,-0.21584,-0.20703,-0.22616,-0.27034,-0.33488,-0.41375,-0.5,-0.58625,-0.66512,-0.72966,-0.77384,-0.79297,-0.78416,-0.74678,-0.68288,-0.59768,-0.5,-0.5,-0.40082,-0.21955,-0.14336,-0.079102,-0.02744,0.011672,0.03888,0.05532,0.0625,0.062193,0.05632,0.046841,0.03564,0.024414,0.01456,0.0070622,0.00238,0.00033531,0]
    q_data2=[0, -0.00025909,-0.001856,-0.0055679,-0.011632,-0.019824,-0.029538,-0.039863,-0.049664,-0.057659,-0.0625,-0.062848,-0.057456,-0.045244,-0.025382,0.0026367,0.038912,0.08306,0.13414,0.19055,0.25,0.30945,0.36586,0.41694,0.46109,0.49736,0.52538,0.54524,0.55746,0.56285,0.5625,0.55766,0.54966,0.53986,0.52954,0.51982,0.51163,0.50557,0.50186,0.50026,0.5,0.50055,0.504,0.51222,0.52611,0.5457,0.57031,0.59866,0.62902,0.65938,0.6875,0.71113,0.7281,0.73645,0.73461,0.72148,0.69661,0.66029,0.61372,0.55916,0.5,0.5,0.38827,0.34582,0.31645,0.30137,0.30054,0.31288,0.33642,0.36856,0.40625,0.44619,0.48502,0.51957,0.547,0.56504,0.57219,0.56792,0.55287,0.52903,0.5,0.5,0.46984,0.40655,0.37274,0.3375,0.30115,0.26419,0.22723,0.191,0.15625,0.12375,0.094208,0.068257,0.046386,0.028906,0.015904,0.0071955,0.002282,0.00030475, -4.4409e-16]
    q_data3=[0,-0.0001076,-0.0007756,-0.0023439,-0.0049392,-0.0085059,-0.012836,-0.017599,-0.022374,-0.026679,-0.03,-0.031823,-0.031666,-0.029103,-0.023804,-0.015557,-0.0043008,0.0098414,0.026536,0.045205,0.065,0.084795,0.10346,0.12016,0.1343,0.14556,0.1538,0.1591,0.16167,0.16182,0.16,0.15668,0.15237,0.1476,0.14284,0.13851,0.13494,0.13234,0.13078,0.13011,0.13,0.12991,0.12933,0.12796,0.12565,0.12238,0.11828,0.11356,0.1085,0.10344,0.09875,0.094812,0.091984,0.090591,0.090898,0.093086,0.097232,0.10329,0.11105,0.12014,0.13,0.13,0.14773,0.15295,0.15465,0.15257,0.14678,0.13768,0.12588,0.11218,0.0975,0.082821,0.069122,0.05732,0.048216,0.042432,0.040349,0.042052,0.047268,0.055307,0.065,0.065,0.074785,0.089985,0.094003,0.095186,0.093502,0.089123,0.082382,0.073742,0.06375,0.05301,0.042138,0.031729,0.022318,0.014346,0.0081168,0.0037666,0.0012224,0.00016672,1.1102e-16]

   
    q_output=[None]*len(q_data1)
    qd_output=[None]*len(q_data1)
    torque_output=[None]*len(q_data1)

    while not rospy.is_shutdown():
      if i>len(q_data1)-1:
        break
      joint_msg.position = [q_data1[i], q_data2[i], q_data3[i]]
      joint_msg.velocity = []  
      joint_msg.effort = []
      joint_pub.publish(joint_msg)

      q_output[i]=joint_sub.position
      qd_output[i]=joint_sub.velocity
      torque_output[i]=joint_sub.effort

      rospy.sleep(1/float(ra))
      i+=1
    rate.sleep()
    
    #Print Code
    with open('data_position.csv', 'wb') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
        for i in range(0,len(q_data1)):
          wr.writerow(q_output[i])
    with open('data_velocity.csv', 'wb') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
        for i in range(0,len(q_data1)):
          wr.writerow(qd_output[i])
    with open('data_torque.csv', 'wb') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
        for i in range(0,len(q_data1)):
          wr.writerow(torque_output[i])    
    

if __name__ == '__main__':
    try:
      main(r)
      
    except rospy.ROSInterruptException:
      
      pass
