#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((-0.1016 , -0.1016, 0.4300),
                         (0.0, 0.0, 1.0, 0.0),
                         rospy.Time.now(), 
                         "PSM1_0", "world"
                         )
        br.sendTransform((0.0896, 0, 0),
                         (0, 0.0, 0.0, 1),
                         rospy.Time.now(),
                         "PSM1_1",
                         "PSM1_0")
	br.sendTransform((0, 0, 0.4166),
                         (0, 0.0, 0.0, 1),
                         rospy.Time.now(),
                         "PSM1_2",
                         "PSM1_1")
	br.sendTransform((0.4318, 0, 0.1429),
                         (0, 0.0, 0.0, 1),
                         rospy.Time.now(),
                         "PSM1_3",
                         "PSM1_2")
	br.sendTransform((0.4318, 0, -0.1302),
                         (0, 0.0, 0.7071, 0.7071),
                         rospy.Time.now(),
                         "PSM1_4",
                         "PSM1_3")
	br.sendTransform((0, -0.4089, 0),
                         (0.7071, 0, 0, 0.7071),
                         rospy.Time.now(),
                         "PSM1_5",
                         "PSM1_4")
	br.sendTransform((0, -0.1029, 0),
                         (-0.5, -0.5, -0.5, 0.5),
                         rospy.Time.now(),
                         "PSM1_6",
                         "PSM1_5")
        br.sendTransform((0.4864, 0, 0.1524),
                         (0, 0, -0.7071, 0.7071),
                         rospy.Time.now(),
                         "PSM1_Tip",
                         "PSM1_6")



	br.sendTransform((0.1016, -0.1016, 0.43),
                         (0, 0.0, 0.0, 1),
                         rospy.Time.now(),
                         "PSM2_0","world")
	br.sendTransform((0.0896, 0, 0),
                         (0, 0, 0, 1),
                         rospy.Time.now(),
                         "PSM2_1","PSM2_0")        
	br.sendTransform((0, 0, 0.4166),
                         (0, 0.0, 0.0, 1),
                         rospy.Time.now(),
                         "PSM2_2",
                         "PSM2_1")
	br.sendTransform((0.4318, 0, 0.1429),
                         (0, 0.0, 0.0, 1),
                         rospy.Time.now(),
                         "PSM2_3",
                         "PSM2_2")
	br.sendTransform((0.4318, 0, -0.1302),
                         (0, 0.0, 0.7071, 0.7071),
                         rospy.Time.now(),
                         "PSM2_4",
                         "PSM2_3")
	br.sendTransform((0, -0.4089, 0),
                         (0.7071, 0, 0, 0.7071),
                         rospy.Time.now(),
                         "PSM2_5",
                         "PSM2_4")
	br.sendTransform((0, -0.1029, 0),
                         (-0.5, -0.5, -0.5, 0.5),
                         rospy.Time.now(),
                         "PSM2_6",
                         "PSM2_5")
        br.sendTransform((0.4864, 0, 0.1524),
                         (0, 0, -0.7071, 0.7071),
                         rospy.Time.now(),
                         "PSM2_Tip",
                         "PSM2_6")


        br.sendTransform((0, 0, 0.1264),
                         (0, 0, 0.7071, 0.7071),
                         rospy.Time.now(),
                         "PSM3_0","world")
        br.sendTransform((0.0896, 0, 0),
                         (0, 0, 0, 1),
                         rospy.Time.now(),
                         "PSM3_1","PSM3_0")
	br.sendTransform((0, 0, 0.3404),
                         (0, 0.0, 0.0, 1),
                         rospy.Time.now(),
                         "PSM3_2",
                         "PSM3_1")
	br.sendTransform((0.5842,0, 0.1429),
                         (0, 0.0, 0.0, 1),
                         rospy.Time.now(),
                         "PSM3_3",
                         "PSM3_2")
	br.sendTransform((0.4318, 0, 0.2571),
                         (0, 0.0, 0.7071, 0.7071),
                         rospy.Time.now(),
                         "PSM3_4",
                         "PSM3_3")
	br.sendTransform((0, -0.4089, 0),
                         (0.7071, 0, 0, 0.7071),
                         rospy.Time.now(),
                         "PSM3_5",
                         "PSM3_4")
	br.sendTransform((0, -0.1029, 0),
                         (-0.5, -0.5, -0.5, 0.5),
                         rospy.Time.now(),
                         "PSM3_6",
                         "PSM3_5")
        br.sendTransform((0.4864, 0, 0.1524),
                         (0, 0, -0.7071, 0.7071),
                         rospy.Time.now(),
                         "PSM3_Tip",
                         "PSM3_6")


        br.sendTransform((0, 0, 0.43),
                         (0, 0, 0.7071, 0.7071),
                         rospy.Time.now(),
                         "ECM_0","world")
        br.sendTransform((0.0896, 0, 0),
                         (0, 0, 0, 1),
                         rospy.Time.now(),
                         "ECM_1","ECM_0")
	br.sendTransform((0, 0, 0.4166),
                         (0, 0.0, 0.0, 1),
                         rospy.Time.now(),
                         "ECM_2",
                         "ECM_1")
	br.sendTransform((0.4318, 0, 0.1429),
                         (0, 0.0, 0.0, 1),
                         rospy.Time.now(),
                         "ECM_3",
                         "ECM_2")
	br.sendTransform((0.4318, 0, -0.3459),
                         (0, 0.0, 0.7071, 0.7071),
                         rospy.Time.now(),
                         "ECM_4",
                         "ECM_3")
	br.sendTransform((0, 0, 0),
                         (-0.2706, 0.2706, 0.6533, 0.6533),
                         rospy.Time.now(),
                         "ECM_5",
                         "ECM_4")
	br.sendTransform((-0.0667, 0, 0),
                         (0, 0, 0, 1),
                         rospy.Time.now(),
                         "ECM_6",
                         "ECM_5")
        br.sendTransform((0, 0, 0.1016),
                         (0, 0, 0.7071, 0.7071),
                         rospy.Time.now(),
                         "ECM_Tip",
                         "ECM_6")	
rate.sleep()
