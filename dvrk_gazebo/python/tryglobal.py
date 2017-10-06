#!/usr/bin/env python
# http://wiki.ros.org/rospy/Overview/Messages

#rostopic pub /dvrk/PSM1/set_robot_state std_msgs/String DVRK_POSITION_CARTESIAN
# rostopic info /dvrk/MTML/set_position_joint


import math
import time
import csv


def variable():
  global q_data1
  global q_data2
  global q_data3
  q_data1= [0,-0.01522,-0.10144,-0.28026,-0.53248]   
  q_data2= [0,0.00152,0.01152,0.03672,0.08192]
  q_data3= [0.13,0.12867,0.1213,0.10656,0.086992,0.0675,0.053968,0.051796,0.064464,0.092092,0.13,0.16735,0.19177,0.1976,0.1854,0.16,0.12864,0.099036,0.077469,0.066888,0.065,0.064334,0.060648,0.053282,0.043496,0.03375,0.026984,0.025898,0.032232,0.046046,0.065,0.065,0.094003,0.093502,0.082382,0.06375,0.042138,0.022318,0.0081168,0.0012224,0]   
    
  q_data1=q_data2

def written():
       
   with open('q.csv', 'wb') as myfile:
      wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
      for i in range(0,len(q_data1)):
        wr.writerow(q_data1)

   with open('q2.csv', 'wb') as myfile:
      wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
      for i in range(0,len(q_data1)):
        wr.writerow(q_data2)
   with open('q3.csv', 'wb') as myfile:
      wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
      for i in range(0,len(q_data3)):
        wr.writerow(q_data3)
variable()
written()
    