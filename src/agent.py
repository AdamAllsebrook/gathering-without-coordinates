#!/usr/bin/env python3

import rospy
import numpy as np
import time

from geometry_msgs.msg import TwistStamped, Vector3, Pose2D
from miro2.lib import wheel_speed2cmd_vel

from gathering_without_coordinates.srv import GetMiRoPos


def size(vec):
    return np.sqrt(vec[0] ** 2 + vec[1] ** 2)


class Agent:
    TICK = 10
    RANGE = np.Infinity

    def __init__(self):
        rospy.init_node('agent')

        self.name = rospy.get_namespace()

        rospy.wait_for_service('/get_miro_pos')
        self.get_miro_pos = rospy.ServiceProxy('/get_miro_pos', GetMiRoPos)

        self.pub_vel = rospy.Publisher(
            self.name + 'control/cmd_vel',
            TwistStamped,
            queue_size=0
        )

        self.sub_pose = rospy.Subscriber(
            self.name + 'sensors/body_pose',
            Pose2D,
            self.callback_miro_pose
        )

        self.rate = rospy.Rate(self.TICK)
        self.pose = Pose2D()

        self.state = 'turn'
        self.timer = 0

    def callback_miro_pose(self, msg):
        self.pose = msg

    # returns a Vector3 list of each miros relative position
    def get_relative_positions(self):
        res = self.get_miro_pos(self.name.replace('/', ''), self.RANGE)
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
            print(self.state)
            if self.state == 'turn':
                
                relative_positions = self.get_relative_positions()
                if len(relative_positions) > 0:
                    sum_all = Vector3()
                    for pos in relative_positions:
                        sum_all.x += pos.x
                        sum_all.y += pos.y
                    centre = Vector3(
                        sum_all.x / len(relative_positions),
                        sum_all.y / len(relative_positions),
                        0
                    )
                    # pose = np.array([np.cos(self.pose.theta), np.sin(self.pose.theta)])

                    # print(self.pose.theta)

                    theta = (np.arctan2(centre.y, centre.x) - self.pose.theta) % (2 * np.pi)
                    

                    # print(np.pi - 0.5, theta, np.pi + 0.5)
                    if (np.pi - 0.1 < theta < np.pi + 0.1) or -0.1 < theta < 0.1:
                        self.state = 'move'
                        self.timer = time.time()
  
                    elif theta > np.pi:
                        self.drive(2, -0)
                    else:
                        self.drive(-0, 2)


            elif self.state == 'move':
                self.drive(3, 3)
                if time.time() - self.timer > 5:
                    self.state = 'turn'

            # diff = np.arccos(np.dot(centre, pose) / np.abs(size(centre) * size(pose)))

            # print(diff)
            # if diff > np.pi:
            #     self.drive(0, 1)
            # else:
            #     self.drive(1, 0)
            

            # msg_cmd_vel = TwistStamped()
            # dtheta = np.arctan2(centre.y, centre.x) - self.pose.theta
            # print(self.name, dtheta)
            # dr = 3
            # msg_cmd_vel.twist.linear.x = dr
            # msg_cmd_vel.twist.angular.z = dtheta
            # self.pub_vel.publish(msg_cmd_vel)

            self.rate.sleep()


if __name__ == '__main__':
    main = Agent()
    main.loop()