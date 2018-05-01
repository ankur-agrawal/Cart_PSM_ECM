#!/usr/bin/env python

from dvrk_library import dvrk_library as dvrk
import time

a=dvrk.init()
# a.i()
i=0
while a.quit():
    i=1+1
    a.update()
    a.teleop();
    time.sleep(1)
    # print i
    # rospy.sleep(0.003)
# a.exit_thread()
# raw_input("press enter")
