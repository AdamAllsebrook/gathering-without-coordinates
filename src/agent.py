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
    RANGE = 2

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
        self.move_time = 0

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

    def turn_to_angle(self, theta):
        while not rospy.is_shutdown():

            diff = (theta - self.pose.theta) % (2 * np.pi)
            if (np.pi - 0.1 < diff < np.pi + 0.1) or -0.1 < diff < 0.1:
                return

            elif diff > np.pi:
                self.drive(2, -0)
            else:
                self.drive(-0, 2)

            self.rate.sleep()

    def loop(self):
        while not rospy.is_shutdown():
            relative_positions = self.get_relative_positions()

            for pos in relative_positions:
                # print(size(np.array([pos.x, pos.y]) - np.array([self.pose.x, self.pose.y]))
                if size(np.array([pos.x, pos.y]) - np.array([self.pose.x, self.pose.y])) < 1:
                    self.state = 'fleet'
                    break

            if self.state == 'turn':
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
                    theta = (np.arctan2(centre.y, centre.x) - self.pose.theta) % (2 * np.pi)
                    
                    if (np.pi - 0.1 < theta < np.pi + 0.1) or -0.1 < theta < 0.1:
                        self.state = 'move'
                        self.timer = time.time()
                        self.move_time = 5
  
                    elif theta > np.pi:
                        self.drive(2, -0)
                    else:
                        self.drive(-0, 2)

                else:
                    # explore
                    theta = np.random.random() * 2 * np.pi
                    self.turn_to_angle(theta)
                    self.state = 'move'
                    self.timer = time.time()
                    self.move_time = 5

            elif self.state == 'move':
                self.drive(3, 3)
                if time.time() - self.timer > self.move_time:
                    self.state = 'turn'

            elif self.state == 'fleet':
                print(self.name, 'im fleeting :)')
                pose = 0
                n = 0
                for pos in relative_positions:
                    if size(np.array([pos.x, pos.y]) - np.array([self.pose.x, self.pose.y])) < 1:
                        pose += pos.z
                        n += 1
                pose = pose / n

                self.turn_to_angle(pose)
                self.state = 'move'
                self.timer = time.time()
                self.move_time = 5



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