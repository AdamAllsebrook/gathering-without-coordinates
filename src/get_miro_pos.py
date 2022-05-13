#!/usr/bin/env python3

import rospy
import numpy as np
from joblib import dump, load
import os
import sys

from gazebo_msgs.msg import ModelStates, LinkStates
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

from gathering_without_coordinates.srv import GetMiRoPos, GetMiRoPosResponse

dir_path = os.path.dirname(os.path.realpath(__file__))

RESULTS_PATH = dir_path + '/results.gz'

if sys.argv[1] == 'true':
    WALLS = True
else:
    WALLS = False


# get the 2D distance between two Vector3
def distance(pos1, pos2):
    return np.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)

# get the determinant of a 2x2 matrix
def det(mat):
    return mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0]


class GetMiRoPosService:
    data = None

    def __init__(self):
        rospy.init_node('get_miro_pos_server')

        # init server
        self.server = rospy.Service(
            'get_miro_pos',
            GetMiRoPos,
            self.callback_get_miro_pos
        )

        # get state of all models - used to get miro positions
        self.sub_model_states = rospy.Subscriber(
            'gazebo/model_states',
            ModelStates,
            self.callback_model_states
        )

        # store miro positions when we see them
        self.model_states = {}
        
        # hardcoded position and orientation of all walls in the maze - all are the same length
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

        # load data file to store results
        if os.path.exists(RESULTS_PATH):
            self.data = load(RESULTS_PATH)
        else:
            self.data = []
        self.data.append(np.empty((0, 2)))

        # get start time 
        self.start = rospy.get_rostime()

    # test if vision is blocked by a wall
    # uses Cramers rule to work out intersection using matrix dets
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
            #       or
            #     x + b * dx,
            #     y + b * dy
            # )

            # intersection assumes both lines are infinitely long, we can use size of a to only check with the wall length
            if -self.wall_length / 2 < a < self.wall_length / 2:
                return True
        return False


    # get relative miro positions
    def callback_get_miro_pos(self, req):
        relative_positions = []
        res = GetMiRoPosResponse()
        if req.name not in self.model_states:
            return res
        # get position of requesting miro
        req_pos = self.model_states[req.name]

        # get relative positions of other miros within range, that are not blocked by walls
        for name, position in self.model_states.items():
            if name != req.name and distance(req_pos, position) < req.range and not (WALLS  and self.vision_blocked(
                req_pos.x, req_pos.y, position.x - req_pos.x, position.y - req_pos.y
            )):
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
        # stop running after 3 minutes (length of test)
        if rospy.get_rostime().to_sec() > 180:
            print('\n\n\n3 minutes are up! Data index is %d\n\n\n' % (len(self.data) - 1))
            rospy.signal_shutdown('')

        # update the position of all miros
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
        # store the average distance at this point in time
        self.save_average_distance()

    # store the average distance between all miros
    def save_average_distance(self):
        # check if everything is loaded
        if self.data is None or len(self.model_states) < 3: return

        # if there is not data yet, or 0.1 seconds have passed since the last data point
        if self.data[-1].shape[0] == 0 or self.data[-1][-1][0] + 0.1 < rospy.get_rostime().to_sec():
            total = 0
            n = 0
            for name in self.model_states:
                for other_name in self.model_states:
                    if name != other_name:
                        total += np.sqrt((self.model_states[name].x - self.model_states[other_name].x) ** 2 + (self.model_states[name].y - self.model_states[other_name].y) ** 2)
                        n += 1

            if n > 0:
                dist = total / n
                time = rospy.get_rostime() - self.start
                time = time.to_sec()

                # store time and average distance
                self.data[-1] = np.append(self.data[-1], np.array([[time, dist]]), axis=0)

    # save the data to a file
    def save(self):
        dump(self.data, dir_path + '/results.gz')

if __name__ == '__main__':
    main = GetMiRoPosService()
    rospy.on_shutdown(main.save)
    rospy.spin()