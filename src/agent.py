#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import TwistStamped
from miro2.lib import wheel_speed2cmd_vel

from gathering_without_coordinates.srv import GetMiRoPos


class Agent:
    TICK = 10
    RANGE = np.Infinity

    def __init__(self):
        rospy.init_node('agent')

        self.name = rospy.get_namespace()

        rospy.wait_for_service('get_miro_pos')
        self.get_miro_pos = rospy.ServiceProxy('get_miro_pos', GetMiRoPos)

        self.rate = rospy.Rate(self.TICK)

    # returns a Vector3 list of each miros relative position
    def get_relative_positions(self):
        res = self.get_miro_pos(self.name, self.RANGE)
        return res.relative_positions

    def drive(self, speed_l=0.1, speed_r=0.1):  # (m/sec, m/sec)
        """
        Wrapper to simplify driving MiRo by converting wheel speeds to cmd_vel
        """
        # Prepare an empty velocity command message
        msg_cmd_vel = TwistStamped()

        # Desired wheel speed (m/sec)
        wheel_speed = [speed_l, speed_r]

        # Convert wheel speed to command velocity (m/sec, Rad/sec)
        (dr, dtheta) = wheel_speed2cmd_vel(wheel_speed)

        # Update the message with the desired speed
        msg_cmd_vel.twist.linear.x = dr
        msg_cmd_vel.twist.angular.z = dtheta

        # Publish message to control/cmd_vel topic
        self.pub_vel.publish(msg_cmd_vel)

    def loop(self):
        while not rospy.is_shutdown():
            self.drive(0.5, 0.5)
            self.rate.sleep()


if __name__ == '__main__':
    main = Agent()
    main.loop()