'''
Path Planning on Polaris GEM using MCTS
Matthew Chang, Kazuki Shin, Gabrielle Chen
CS 598: MAAV Fall 2019
'''

import math
import random
import time
import numpy as np
import cv2 as cv
from scipy.spatial import cKDTree

planning_horizon = 25 # Create a black image
C = math.sqrt(2) * 300
S = 1
T = 1
W = 1.676
turn_rad_sec = 4
max_wheel_angle = 0.600
max_steer_angle = 3.5 * np.pi
wheel_angle_per_steer_angle = max_wheel_angle / max_steer_angle
possible_angles = np.linspace(-T*turn_rad_sec, T*turn_rad_sec, num=3)
scale_factor = 20
point_radius = 0.2
delay_steps = 0

def clip(val,low,high):
    return max(min(val,high),low)

def forward(pos, heading, steer_angle, s, t, angle):
    '''
    @param pos - position
    @para heading - heading in radians to straight vertical
    @param s - speed
    @param t - timestep
    @param max_steer_angle - angle of wheels
    '''
    steer_angle += angle    # apply wheel angle change
    steer_angle = clip(steer_angle,-max_steer_angle,max_steer_angle)

    wheel_angle = wheel_angle_per_steer_angle * steer_angle

    rot = np.array([[math.cos(heading), -math.sin(heading)],
                    [math.sin(heading), math.cos(heading)]])

    if wheel_angle == 0:
        displacement = np.array([0, s * t])
        new_heading = heading
    elif wheel_angle > 0:
        r = W / math.sin(wheel_angle)
        theta = s * t / r   # distance moved around circle in radians
        displacement = np.array([math.cos(theta) - 1, math.sin(theta)]) * r     # rotation matrix from heading
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
        closest_dist,_ = obstacles.query(self.pos)
        self.dist_from_closest = closest_dist
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
            return 2*self.cum_goodness + 0.1*self.cum_pos
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
    img = np.zeros((512, 512, 3), np.uint8)
    node = Node(np.array([0, 0]), speed, 0, steer_angle, 0, None)

    points = points[np.random.randint(points.shape[0], size=(1000, )),:]
    points = np.matmul(points, np.array([[0, 1], [-1, 0]])) #rotate 90 degrees counterclokwise
    start = time.time()
    tree = cKDTree(points)
    [node.rollout(tree) for _ in range(250)]
    rows = np.random.randint(points.shape[0], size=(200, ))
    draw = points[rows, :]
    for o1 in draw:
        cv.circle(img, tuple(pos_shift(np.array(o1))),
                  int(point_radius * scale_factor), (255, 0, 0), -1)

    n = node
    for x in range(planning_horizon):
        drawState(img, to_state(n))
        print(to_state(n))
        print(n.cum_goodness)
        n = n.best_rollout[1]
        if n == None:
            break
    return img, node


def planner_test():
    np.random.seed(0)
    data = np.load('gab.npy')
    data[:, 0] -= 4
    data[:, 1] -= 2
    img, node = display_plan(data,0)

    node.visits
    node.visits
    node.children[0].visits
    node.children[2].visits
    node.children[1].visits
    node.children[0].cum_path_min
    node.children[1].value_ratio()
    node.children[2].value_ratio()

    next_node = node.best_rollout[1]
    current_steer_angle = 0
    max_change = T*turn_rad_sec
    diff = max(min(next_node.control_value,max_change),-max_change)
    next_step = diff + current_steer_angle

    cv.imshow("img", img)
    cv.waitKey(0)
