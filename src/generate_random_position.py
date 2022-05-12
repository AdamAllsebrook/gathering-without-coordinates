#!/usr/bin/env python3
import random
import rospy
x = random.randrange(-8, 9) / 4
y = random.randrange(-8, 9) / 4
rospy.set_param('~pos', '-x %f -y %f' % (x, y))