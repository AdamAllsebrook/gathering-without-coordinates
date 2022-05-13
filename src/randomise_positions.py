#!/usr/bin/env python3

# not used in the final implementation of tests

import rospy
import random
import numpy as np

from tf.transformations import quaternion_about_axis
from geometry_msgs.msg import Pose, Quaternion
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState

def callback_model_states(msg):
    global miros
    # print(msg.name)
    for name in msg.name:
        if 'miro' in name and name not in miros:
            miros.append(name)

miros = []
rospy.sleep(2)

sub_model_states = rospy.Subscriber(
    '/gazebo/model_states',
    ModelStates,
    callback_model_states
)
# rospy.wait_for_service('/gazebo/set_model_state')
set_model = rospy.ServiceProxy(
    '/gazebo/set_model_state', SetModelState)

rospy.init_node('randomise_positions')


while len(miros) < 3:
    rospy.sleep(0.1)

for miro in miros:
    pose = Pose()
    pose.position.x = random.randrange(-8, 9) / 4
    pose.position.y = random.randrange(-8, 9) / 4

    theta = random.random() * 2 * np.pi
    (x, y, z, w) = quaternion_about_axis(theta, (0, 0, 1))
    pose.orientation = Quaternion(x, y, z, w)
    
    model_state = ModelState()
    model_state.model_name = miro
    model_state.pose = pose
    res = set_model(model_state)

    print('set %s to x:%.3f, y:%.3f, theta:%.3f' % (miro, pose.position.x, pose.position.y, np.degrees(theta)))