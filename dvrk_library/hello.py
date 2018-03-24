#!/usr/bin/env python

from dvrk_library import dvrk_library as dvrk

a=dvrk.init()
# a.i()

for i in range(1000000):
    a.update(i/1000000.0)
    # print i
a.exit_thread()
# raw_input("press enter")
