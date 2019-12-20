# ---------------------------------------------
# need to graph time vs steering angle under a given control value
# try with several values and hope it's consistently linear
# same with speed, but maybe can just fix a speed and not fuck with it
import numpy as np
import cv2 as cv
import math
import random
from scipy.spatial import cKDTree
import time
# from steer_test import sendSteer

# Create a black image
planning_horizon = 25
C = math.sqrt(2) * 300 

# possible_angles = np.array([np.pi/10])
# possible_angles = np.array([np.pi/10])

S = 1
T = 1
W = 1.676
turn_rad_sec = 4
# max_wheel_angle = 0.9057
max_wheel_angle = 0.600
max_steer_angle = 3.5 * np.pi
wheel_angle_per_steer_angle = max_wheel_angle / max_steer_angle
possible_angles = np.linspace(-T*turn_rad_sec, T*turn_rad_sec, num=3)

scale_factor = 20
point_radius = 0.2
delay_steps = 0

def clip(val,low,high):
    return max(min(val,high),low)

#position
# heading in radians to straight vertical
# speed
# timestep
# angle of wheels
def forward(pos, heading, steer_angle, s, t, angle):
    # apply wheel angle change
    # max_angle_change = turn_rad_sec * t
    # diff = max(min(angle, max_angle_change), -max_angle_change)
    # diff = clip(angle,-max_angle_change,max_angle_change)
    steer_angle += angle
    # steer_angle = max(min(steer_angle, max_steer_angle),-max_steer_angle)
    steer_angle = clip(steer_angle,-max_steer_angle,max_steer_angle)

    wheel_angle = wheel_angle_per_steer_angle * steer_angle

    rot = np.array([[math.cos(heading), -math.sin(heading)],
                    [math.sin(heading), math.cos(heading)]])

    if wheel_angle == 0:
        displacement = np.array([0, s * t])
        new_heading = heading
    elif wheel_angle > 0:
        r = W / math.sin(wheel_angle)
        # distance moved around circle in radians
        theta = s * t / r
        # rotation matrix from heading
        displacement = np.array([math.cos(theta) - 1, math.sin(theta)]) * r
        new_heading = math.fmod(heading + theta, np.pi * 2)
    else:
        r = W / math.sin(-wheel_angle)
        theta = s * t / r
        displacement = np.array([-(math.cos(theta) - 1), math.sin(theta)]) * r
        new_heading = math.fmod(heading - theta, np.pi * 2)
    new_pos = np.matmul(rot, displacement) + pos

    return new_pos, new_heading, steer_angle


def unitVector(angle):
    ang = angle + np.pi / 2
    return np.array([math.cos(ang), math.sin(ang)])


def collide(rect, point):
    return np.linalg.norm(point - rect) < 1
    # return not (point[0] < min(rect[0][0], rect[1][0])
    # or point[0] > max(rect[0][0], rect[1][0])
    # or point[1] < min(rect[0][1], rect[1][1])
    # or point[1] > max(rect[0][1], rect[1][1]))


def random_obs():
    po = np.random.randint(-10, 10, size=(2, ))
    po[1] += 10
    return po


class Node:
    def __init__(self,
                 pos,
                 speed,
                 heading,
                 steer_angle,
                 depth,
                 control_value,
                 parent=None,
                 dist_from_closest=0):
        self.pos = pos
        self.speed = speed
        self.heading = heading
        self.steer_angle = steer_angle
        self.children = []
        self.visits = 0
        self.value = 0
        self.depth = depth
        self.best_rollout = (-np.inf, None)
        self.control_value = control_value
        self.parent = parent
        self.dist_from_closest = dist_from_closest

    def select_child(self):
        if len(self.children) == 0:
            for ang in possible_angles:
                self.children.append(self.node_from_control(ang))

        no_expand = list(filter(lambda x: x.visits == 0, self.children))
        if len(no_expand) > 0:
            return random.choice(no_expand)

        values = [
            c.value / c.visits +
            C * math.sqrt(math.log(self.visits) / c.visits)
            for c in self.children
        ]
        return self.children[np.argmax(values)]

    def value_ratio(self):
        return self.value / self.visits

    def rollout(self, obstacles):
        # if self.collides(obstacles) or self.depth == planning_horizon:
        closest_dist,_ = obstacles.query(self.pos)
        self.dist_from_closest = closest_dist
        # self.cum_goodness = min(closest_dist,2)
        # self.cum_goodness = -1/(closest_dist/2)
        if closest_dist < 1:
            self.cum_goodness = -100
        elif closest_dist < 2.0:
            self.cum_goodness = -10
        else:
            self.cum_goodness = 0

        if self.parent is not None:
            self.cum_path_min = min(self.parent.cum_path_min,closest_dist)
            self.cum_goodness += self.parent.cum_goodness
            self.cum_pos = self.pos[1] + self.parent.cum_pos
        else:
            self.cum_path_min = closest_dist
            self.cum_pos = self.pos[1]
        if self.depth == planning_horizon or closest_dist < 1:
            # return self.depth
            # print(self.pos[1],self.depth,self.min_penalty())
            # return 0.5*self.pos[1] + 2.0*self.depth + 100.0 * self.min_penalty()
            # return 0.5*self.pos[1] + 50.0 * self.cum_path_min
            # return 0.5*self.cum_pos + 50.0 * self.cum_goodness
            return 2*self.cum_goodness + 0.1*self.cum_pos
            # return 100*self.cum_path_min
        else:
            self.visits = self.visits + 1
            child = self.select_child()
            new_value = child.rollout(obstacles)
            if new_value >= self.best_rollout[0]:
                self.best_rollout = (new_value, child)
            self.value = self.value + new_value
            return new_value

    def cumulative_penalty(self):
        if self.parent is None:
            return 0
        return cumulative_penalty(self.parent) +  self.dist_from_closest

    def min_penalty(self):
        if self.parent is None:
            return float("inf")
        return min(self.parent.min_penalty(), self.dist_from_closest)

    def collides(self, obstacles):
        dist, ii = obstacles.query(self.pos)
        return dist < point_radius

    def node_from_control(self, ang):
        control_node = self
        for _ in range(0, delay_steps):
            control_node = control_node.parent if control_node is not None else None
        control = 0
        if control_node is not None and control_node.control_value is not None:
            control = control_node.control_value

        npos, nheading, nsa = forward(self.pos, self.heading, self.steer_angle,
                                      S, T, control)
        return Node(npos,
                    self.speed,
                    nheading,
                    nsa,
                    self.depth + 1,
                    ang,
                    parent=self)


# node = Node(np.array([0, 0]), S, 0, 0)
# obstacles = [(0, 3)]
# obstacles = [((200, 200), (300, 300))]
# obstacles = []
# obstacles = [((100, 200), (300, 250)),((330, 200), (400, 300))]

# obstacles = np.load('points.npy')[:,[0,1]]
# #rotate 90 degrees counterclokwise
# obstacles = np.matmul(obstacles,np.array([[0,1],[-1,0]]))
# from scipy.spatial import cKDTree
# # import pdb; pdb.set_trace()
# import time
# start = time.time()
# tree = cKDTree(obstacles)
# # obstacles = [random_obs() for _ in range(0, 10)]
# [node.rollout(tree) for _ in range(250)]
# print(time.time()-start)

# node.visits
# node.value
# node.select_child().select_child()


def pos_shift(pos):
    pts = pos.copy()
    pts *= scale_factor
    pts[1] = -pts[1]
    pts += [300, 400]
    return np.round(pts).astype(np.int)


def drawState(img, state):
    pos, heading = state
    heading = -heading
    pos = pos_shift(pos)
    offset = (7 * unitVector(heading)).astype(np.int)
    st = pos.astype(np.int)
    cv.line(img, tuple(st), tuple(st + offset), (0, 255, 0), 3)

def to_state(node):
    return (node.pos, node.heading)

def display_plan(points, steer_angle, speed=S):
    # np.save("gab",points)
    img = np.zeros((512, 512, 3), np.uint8)
    node = Node(np.array([0, 0]), speed, 0, steer_angle, 0, None)

    points = points[np.random.randint(points.shape[0], size=(1000, )),:]
    #rotate 90 degrees counterclokwise
    points = np.matmul(points, np.array([[0, 1], [-1, 0]]))
    start = time.time()
    tree = cKDTree(points)
    [node.rollout(tree) for _ in range(250)]
    # print(time.time()-start),size
    rows = np.random.randint(points.shape[0], size=(200, ))
    draw = points[rows, :]
    #draw = points
    for o1 in draw:
        cv.circle(img, tuple(pos_shift(np.array(o1))),
                  int(point_radius * scale_factor), (255, 0, 0), -1)

    n = node
    # for n in node.children:
    for x in range(planning_horizon):
        drawState(img, to_state(n))
        print(to_state(n))
        # print(n.control_value)
        print(n.cum_goodness)
        n = n.best_rollout[1]
        if n == None:
            break
    return img, node


#steer()
if __name__ == "__main__":
    np.random.seed(0)
    data = np.load('gab.npy')
    data[:, 0] -= 4
    data[:, 1] -= 2
    img, node = display_plan(data,0)
    # nod = node.children[1]
    # for x in range(planning_horizon):
        # nod = nod.best_rollout[1]
        # print(nod.cum_path_min)

    # node.children[2].value_ratio()
    node.visits
    node.visits
    node.children[0].visits
    node.children[2].visits
    node.children[1].visits
    node.children[0].cum_path_min
    node.children[1].value_ratio()
    node.children[2].value_ratio()
    # img, node = display_plan(data, 0)
    next_node = node.best_rollout[1]
    current_steer_angle = 0
    max_change = T*turn_rad_sec
    diff = max(min(next_node.control_value,max_change),-max_change)
    next_step = diff + current_steer_angle
    # import pdb; pdb.set_trace()
    # node.value/node.visits
    # node.visits
    # node.children[0].value_ratio()
    # node.children[0].pos
    # node.children[2].children[2].pos
    # node.children[2].children[2].value_ratio()
    # node.children[2].children[2].children[2].visits
    # node.children[2].children[2].visits
    # node.children[2].children[2].pos
    cv.imshow("img", img)
    cv.waitKey(0)
