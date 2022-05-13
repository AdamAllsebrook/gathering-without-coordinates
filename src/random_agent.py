#!/usr/bin/env python3

from agent import Agent
import rospy

class Random(Agent):
    def loop(self):
        while not rospy.is_shutdown():
            self.explore()
            self.rate.sleep()


if __name__ == '__main__':
    main = Random()
    main.loop()