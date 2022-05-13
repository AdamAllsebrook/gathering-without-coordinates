#!/usr/bin/env python3

from agent import Agent
import rospy

# agent that only ever explores
class Random(Agent):
    def loop(self):
        while not rospy.is_shutdown():
            self.pub_kin.publish(self.kin_joints)
            self.explore()
            self.rate.sleep()


if __name__ == '__main__':
    main = Random()
    main.loop()