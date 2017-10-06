#!/usr/bin/env python
# http://wiki.ros.org/rospy/Overview/Messages
# license removed for brevity

import rospy
import math
import csv
from std_msgs.msg import Float64 
from sensor_msgs.msg import JointState 

r=20 

def callback1(msg): 
  global sub1
  sub1=msg
      
def callback2():
  b=0
def callback3():
  c=0
def talker1(ra):
    pub1 = rospy.Publisher('/psm_joint_1_position_controller/command',Float64, queue_size=10)
    pub2 = rospy.Publisher('/psm_joint_2_position_controller/command',Float64, queue_size=10)
    pub3 = rospy.Publisher('/psm_joint_3_position_controller/command',Float64, queue_size=10)
    rospy.Subscriber('/joint_states', JointState, callback1)
    rospy.init_node('Talkity_talk_talk', anonymous=True)
    rate = rospy.Rate(ra) # 10hz
    

    #hardcode home to zero 
    rospy.sleep(1)
    d=0.05 
    thresh=0.03

    while not rospy.is_shutdown():
      if -thresh<sub1.position[0]<thresh:
        dq1=None  
      elif sub1.position[0]<0: 
        dq1=sub1.position[0]+d/10
      else: 
        dq1=sub1.position[0]-d/10

      if -thresh<sub1.position[1]<thresh: 
        dq2=None  
      elif sub1.position[1]<0: 
        dq2=sub1.position[1]+d
      else: 
        dq2=sub1.position[1]-d

      if -thresh<sub1.position[2]<thresh: 
        dq3=None  
      elif sub1.position[2]<0: 
        dq3=sub1.position[2]+d
      else: 
        dq3=sub1.position[2]-d
      
      if -thresh<sub1.position[0]<thresh and -thresh<sub1.position[1]<thresh  and -thresh<sub1.position[2]<thresh: 
        break
    
      print('q3')
      rospy.loginfo(dq3)
      print('q2')
      rospy.loginfo(dq2)
      print('q1')
      rospy.loginfo(dq1)

      pub1.publish(dq3)
      pub2.publish(dq2)
      pub3.publish(dq1)
      rospy.sleep(1/float(ra))

    rospy.sleep(3)

    #Excitation Trajectory Position Coordinates
    i=0
    q_data1=[0,-0.01094,-0.07248,-0.19872,-0.37376,-0.5625,-0.72144,-0.80948,-0.79872,-0.68526,-0.5,-0.31474,-0.20128,-0.19052,-0.27856,-0.4375,-0.62624,-0.80128,-0.92752,-0.98906,-1,-1.0024,-1.0146,-1.0356,-1.0563,-1.0625,-1.0389,-0.97256,-0.85664,-0.69382,-0.5,-0.5,-0.21584,-0.22616,-0.33488,-0.5,-0.66512,-0.77384,-0.78416,-0.68288,-0.5,-0.5,-0.30618,-0.02744,0.03888,0.0625,0.05632,0.03564,0.01456,0.00238,0]
    q_data2=[0,-0.001856,-0.011632,-0.029538,-0.049664,-0.0625,-0.057456,-0.025382,0.038912,0.13414,0.25,0.36586,0.46109,0.52538,0.55746,0.5625,0.54966,0.52954,0.51163,0.50186,0.5,0.504,0.52611,0.57031,0.62902,0.6875,0.7281,0.73461,0.69661,0.61372,0.5,0.5,0.31645,0.30054,0.33642,0.40625,0.48502,0.547,0.57219,0.55287,0.5,0.5,0.43886,0.30115,0.22723,0.15625,0.094208,0.046386,0.015904,0.002282 -4.4409e-16]
    q_data3=[0,-0.0007756,-0.0049392,-0.012836,-0.022374,-0.03,-0.031666,-0.023804,-0.0043008,0.026536,0.065,0.10346,0.1343,0.1538,0.16167,0.16,0.15237,0.14284,0.13494,0.13078,0.13,0.12933,0.12565,0.11828,0.1085,0.09875,0.091984,0.090898,0.097232,0.11105,0.13,0.13,0.15465,0.14678,0.12588,0.0975,0.069122,0.048216,0.040349,0.047268,0.065,0.065,0.083398,0.093502,0.082382,0.06375,0.042138,0.022318,0.0081168,0.0012224,1.1102e-16]
    q_output=[None]*len(q_data1)
    
    while not rospy.is_shutdown():
      if i>len(q_data1)-1:
      	#i=0
        break

      q1 =q_data1[i]
      q2 =q_data2[i]
      q3 =q_data3[i]
      print('q1')
      rospy.loginfo(q1)
      print('q2')
      rospy.loginfo(q2)
      print('q3')
      rospy.loginfo(q3)

      pub1.publish(q1)
      pub2.publish(q2)
      pub3.publish(q3)
      print(sub1.position)
      q_output[i]=sub1.position
      
      rospy.sleep(0.2) 
      #rospy.sleep(1/float(ra))
      i+=1

    rate.sleep()

    with open('q.csv', 'wb') as myfile:
       wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
       for i in range(0,len(q_data1)):
         wr.writerow(q_output[i])
#def write():
    # with open('q.csv', 'wb') as myfile:
    #   wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
    #   for i in range(0,len(q_data1)):
    #     wr.writerow(q_output[i])
#      with open('qd.csv', 'wb') as myfile:
#        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
#        for i in range(0,len(q_data1)):
#          wr.writerow(q_output2[i])
#      with open('torque.csv', 'wb') as myfile:
#        wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
#        for i in range(0,len(q_data1)):
#          wr.writerow(q_output3[i])    
#      rate.sleep()
		

if __name__ == '__main__':
    try:
        talker1(r)
        #write()
        #talker2(r)
    except rospy.ROSInterruptException:
        pass
