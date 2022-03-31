#!/usr/bin/env python3

import rospy
import numpy as np

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

from gathering_without_coordinates.srv import GetMiRoPos, GetMiRoPosResponse


# get the 2D distance between two Vector3
def distance(pos1, pos2):
    return np.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)


class GetMiRoPosService:

    def __init__(self):
        rospy.init_node('get_miro_pos_server')

        self.server = rospy.Service(
            'get_miro_pos',
            GetMiRoPos,
            self.callback_get_miro_pos
        )

        self.sub_model_states = rospy.Subscriber(
            'gazebo/model_states',
            ModelStates,
            self.callback_model_states
        )
        self.model_states = {}

    # get relative miro positions
    def callback_get_miro_pos(self, req):
        relative_positions = []
        res = GetMiRoPosResponse()
        # get position of requesting miro
        if req.name not in self.model_states:
            return res
        req_pos = self.model_states[req.name]

        # get relative positions of other miros within range
        for name, position in self.model_states.items():
            if name != req.name and distance(req_pos, position) < req.range:
                relative_positions.append(Vector3(
                    position.x - req_pos.x,
                    position.y - req_pos.y,
                    position.z
                ))

        res = GetMiRoPosResponse()
        res.relative_positions = relative_positions
        return res

    # get the current position of all miros
    def callback_model_states(self, msg):
        self.model_states = {}
        for i, model_name in enumerate(msg.name):
            if 'miro' in model_name:
                q = msg.pose[i].orientation
                euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
                self.model_states[model_name] = Vector3(
                    msg.pose[i].position.x,
                    msg.pose[i].position.y,
                    euler[2]
                )


if __name__ == '__main__':
    main = GetMiRoPosService()
    rospy.spin()