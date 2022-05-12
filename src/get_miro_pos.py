#!/usr/bin/env python3

import rospy
import numpy as np

from gazebo_msgs.msg import ModelStates, LinkStates
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

from gathering_without_coordinates.srv import GetMiRoPos, GetMiRoPosResponse


# get the 2D distance between two Vector3
def distance(pos1, pos2):
    return np.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)

def det(mat):
    return mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0]

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

        # self.sub_link_states = rospy.Subscriber(
        #     'gazebo/link_states',
        #     LinkStates,
        #     self.callback_link_states
        # )
        self.model_states = {}
        
        self.walls = [
            (0.344130, 1.836410, 0, 1),
            (1.844120, 1.296410, 1, 0),
            (-1.141240, 1.123330, 0, 1),
            (0, -0.040720, 0, 1),
            (-1.852770, -0.341340, 1, 0),
            (1.275900, -1.072300, 0, 1),
            (0.695900,-1.582300, 1, 0),
            (-1.312770, -1.841330, 0, 1)
        ]
        self.wall_length = 1.05

    def vision_blocked(self, x, y, dx, dy):
        """
        [w_x] + a * [w_dx] = [x] + b * [dx]
        [w_y]       [w_dy]   [y]       [dy]

        
        [w_dx  -dx] [a] = [x - w_x]
        [w_dy  -dy] [b] = [y - w_y]

        A = [w_dx  -dx]
            [w_dy  -dy]

        A1 = [x - w_x  -dx]
             [y - w_y  -dy]

        A2 = [w_dx  x - w_x]
             [w_dy  y - w_y]

        a = det(A1)/det(A)
        b = det(A2)/det(A)
        """
        for i, (w_x, w_y, w_dx, w_dy) in enumerate(self.walls):
            A = np.array([
                [w_dx, -dx],
                [w_dy, -dy]
            ])
            A1 = np.array([
                [x - w_x, -dx],
                [y - w_y, -dy]
            ])
            A2 = np.array([
                [w_dx, x - w_x],
                [w_dy, y - w_y]
            ])

            a = det(A1) / det(A)

            # b = det(A2) / det(A)
            # intersect = (
            #     w_x + a * w_dx,
            #     w_y + a * w_dy,
            #
            #     x + b * dx,
            #     y + b * dy
            # )

            if -self.wall_length / 2 < a < self.wall_length / 2:
                return True
        return False


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
            if name != req.name and distance(req_pos, position) < req.range and not self.vision_blocked(
                req_pos.x, req_pos.y, position.x - req_pos.x, position.y - req_pos.y
            ):
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
                
    # 1.05
    # get the current position of all miros
    # def callback_link_states(self, msg):
    #     for i, link_name in enumerate(msg.name):
    #         if 'inside_walls' in link_name:
    #             print(link_name, msg.pose[i])


if __name__ == '__main__':
    main = GetMiRoPosService()
    rospy.spin()