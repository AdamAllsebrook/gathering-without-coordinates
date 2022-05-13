#!/usr/bin/env python3

import rospy
import numpy as np
import time

from geometry_msgs.msg import TwistStamped, Vector3, Pose2D
from sensor_msgs.msg import Range
from miro2.lib import wheel_speed2cmd_vel
from sensor_msgs.msg import JointState

from gathering_without_coordinates.srv import GetMiRoPos


# return the magnitude of a vector
def size(vec):
    return np.sqrt(vec[0] ** 2 + vec[1] ** 2)


class Agent:
    # ticks per second
    TICK = 10
    # vision range in metres
    RANGE = 5
    # number of miros in the world
    N_MIROS = 3

    def __init__(self):
        rospy.init_node('agent')

        self.name = rospy.get_namespace()

        # service to get relative positions of other miros
        rospy.wait_for_service('/get_miro_pos')
        self.get_miro_pos = rospy.ServiceProxy('/get_miro_pos', GetMiRoPos)

        # publish velocity to move
        self.pub_vel = rospy.Publisher(
            self.name + 'control/cmd_vel',
            TwistStamped,
            queue_size=0
        )

        # subscribe to our pose
        self.sub_pose = rospy.Subscriber(
            self.name + 'sensors/body_pose',
            Pose2D,
            self.callback_miro_pose
        )

        # subscribe to sonar sensor
        self.sub_sonar = rospy.Subscriber(
            self.name + 'sensors/sonar',
            Range,
            self.callback_miro_sonar
        )
        # publish kinematics (head movement)
        self.pub_kin = rospy.Publisher(
            self.name + 'control/kinematic_joints',
            JointState,
            queue_size=0
        )

        # move head down to not look over walls with sonar
        self.head_lift = np.radians(50)
        self.kin_joints = JointState()
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0, self.head_lift, 0, 0]

        self.rate = rospy.Rate(self.TICK)
        self.pose = Pose2D()
        self.sonar = 1

        self.state = self.gather

        rospy.sleep(2)

    def callback_miro_pose(self, msg):
        self.pose = msg

    def callback_miro_sonar(self, msg):
        self.sonar = msg.range

    # returns a Vector3 list of each miros relative position
    def get_relative_positions(self, debug=False):
        res = self.get_miro_pos(self.name.replace('/', ''), self.RANGE)
        if debug:
            print(self.name, ' can see %d other miros' % len(res.relative_positions))
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

    # turn until facing an angle
    def turn_to_angle(self, theta):
        while not rospy.is_shutdown():

            # get difference between current theta and target theta
            diff = (theta - self.pose.theta) % (2 * np.pi)

            # check if at angle (with some leeway)
            if (np.pi - 0.1 < diff < np.pi + 0.1) or -0.1 < diff < 0.1:
                return

            # keep turning
            elif diff > np.pi:
                self.drive(2, -0)
            else:
                self.drive(-0, 2)

            self.rate.sleep()

    # move in a straight line for some time 
    def move_for_time(self, t, speed=1):
        # calculate time to stop moving
        end_time = time.time() + t
        while not rospy.is_shutdown():

            self.drive(speed, speed)

            if time.time() > end_time:
                return
            self.rate.sleep()
        
    # get average x y of a list of Vector3s
    def get_average_position(self, relative_positions):
        sum_all = Vector3()
        for pos in relative_positions:
            sum_all.x += pos.x
            sum_all.y += pos.y
        return Vector3(
            sum_all.x / len(relative_positions),
            sum_all.y / len(relative_positions),
            0
            )

    # fleeting/ flocking
    def fleet(self):
        print(self.name + ' is FLEETING')

        relative_positions = self.get_relative_positions()

        # alignment
        # try to align with the angle of other miros in the flock
        align = self.pose.theta
        n = 0
        for pos in relative_positions:
            if size([pos.x, pos.y]) < 1:
                align += pos.z
                n += 1
        align = align / (n+1)

        # turn to align angle and move forwards
        self.turn_to_angle(align)
        self.move_for_time(5, speed=5)

    # move towards the centre of other miros in vision
    def gather(self):
        print(self.name + ' is GATHERING')
        relative_positions = self.get_relative_positions()
        if len(relative_positions) > 0:
            # get centre of other miros
            centre = self.get_average_position(relative_positions)
            # get angle to move towards that centre
            theta = (np.arctan2(centre.y, centre.x)) % (2 * np.pi)

            # turn to angle and move forwards
            self.turn_to_angle(theta)
            self.move_for_time(5, speed=5)

    # randomly explore in a direction
    def explore(self):
        print(self.name + ' is EXPLORING')
        # get a random angle
        theta = np.random.random() * 2 * np.pi
        # turn to angle and move forwards
        self.turn_to_angle(theta)
        self.move_for_time(5, speed=5)

    # reverse for some time, used to get away from walls
    def reverse(self):
        print(self.name + ' is REVERSING')
        # move backwards and turn a bit
        self.move_for_time(2, -1)
        # self.turn_to_angle(self.pose.theta + np.radians(30))
        # self.move_for_time(5, speed=5)

    # main loop
    def loop(self):
        while not rospy.is_shutdown():
            # set head lift
            self.pub_kin.publish(self.kin_joints)

            # get other miros in vision
            relative_positions = self.get_relative_positions(debug=True)

            # choose a state
            #   if next to wall then reverse
            #   if cant see anyone then explore
            #   if other miro within 1m then fleet
            #   else gather (there must be a miro within vision but not within 1m)
            self.state = self.gather
            if self.sonar < 0.1:
                self.state = self.reverse
            elif len(relative_positions) == 0:
                self.state = self.explore
            else:
                for pos in relative_positions:
                    if size([pos.x, pos.y]) < 1:
                        self.state = self.fleet
                        break

            # carry out state
            self.state()

            self.rate.sleep()


if __name__ == '__main__':
    main = Agent()
    main.loop()