#!/usr/bin/env python

from dvrk_library import dvrk_library as dvrk
import time

a=dvrk.init()
print "initialized"
time.sleep(1)
a.update()
print "updated"
while a.quit():
    a.update()
    # print "updated"
    # a.teleop();
    time.sleep(0.001)
    # print i
    # rospy.sleep(0.003)
# a.exit_thread()
# raw_input("press enter")
