#!/usr/bin/env python3

import numpy as np
import rospy
import tf

from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Quaternion, Pose, Point


if __name__ == '__main__':

    print('waiting for service...')
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    print('$MIRO_DIR_MDK/sim/models/miro_model/miro_model.sdf')
    with open("/home/adam/mdk/sim/models/miro_model/miro_model.sdf", "r") as f:
        product_xml = f.read()

    n_miros = 5
    max_x = 5
    max_y = 5
    poses = np.random.rand(n_miros, 3)

    print('spawning')
    for i in range(n_miros):
        x = (poses[i][0] - 0.5) * max_x
        y = (poses[i][1] - 0.5) * max_y
        q = tf.transformations.quaternion_from_euler(0,0,poses[i][2] * 2 * np.pi)
        orient = Quaternion(q[0], q[1], q[2], q[3])
        pose = Pose(Point(x=x, y=y, z=0), orient)
        spawn_model('miro%d' % i, product_xml, "", pose, "world")
        print('spawned miro', i)