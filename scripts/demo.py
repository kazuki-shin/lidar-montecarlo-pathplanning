'''
Path Planning on Polaris GEM using MCTS
Matthew Chang, Kazuki Shin, Gabrielle Chen
CS 598: MAAV Fall 2019
'''

import sys
package_path='/site-packages35'                       # -- change this path
sys.path.insert(0,package_path)

import os
import numpy as np
import time
import rospy
import cv2

from matplotlib import pyplot as plt
from threading import Lock, Thread

from sensor_msgs.msg import PointCloud2, PointField
from pacmod_msgs.msg import SteeringPIDRpt3,SteeringPIDRpt1,SystemRptFloat
from std_msgs.msg import String, Bool, Float32, Float64

from move_gem import move_forward
from steer_gem import sendSteer
from path_planner import T,turn_rad_sec,max_steer_angle,clip, display_plan
from lidar import pointcloud2_to_array

latest_frame = None
current_steer_angle = 0
kill_thread = 0
current_speed = 0

smooth_value = 0
max_change = T*turn_rad_sec

def render():
    print("render start")
    start = time.time()
    while(True):
        if kill_thread == 1:
            break
        if latest_frame is not None:
            img,node = display_plan(latest_frame,current_steer_angle)
	        next_node = node.best_rollout[1]
            diff = next_node.control_value

            next_step = diff + current_steer_angle
            next_step = clip(next_step,-max_steer_angle,max_steer_angle)
            print("Plan Time: ",time.time() - start)
            start = time.time()
            sendSteer(next_step,turn_rad_sec+1)
            cv2.imshow('plan',img)
            cv2.waitKey(1)

lidar_dir = './data'                                   # -- change this path

def lidar_callback(msg):
    global latest_frame
    timestamp = msg.header.stamp.to_nsec()
    arr= pointcloud2_to_array(msg)

    xs = [row[0] for row in arr]
    ys = [row[1] for row in arr]
    zs = [row[2] for row in arr]
    mat = np.asarray([xs,ys,zs]).T

    # preprocess by mask filter
    mat = mat[(mat[:,2]<0) & (mat[:,2]>-1.5)] # z-axis
    mask = (mat[:,0] < 1.5) & (mat[:,0]>-1.5) & (mat[:,1] < 1) & (mat[:,1] > -1) # x-axis
    mat = mat[np.invert(mask)]
    latest_frame = mat[:,[0,1]]

def steer_callback(msg):
    global current_steer_angle
    current_steer_angle = msg.output

def speed_callback(msg):
    current_speed = msg.data

def montecarlo_demo():
    print( '%s: calling main function ... ' % os.path.basename(__file__))

    if not os.path.exists(lidar_dir):
        os.makedirs(lidar_dir)

    rospy.init_node('velodyne_subscriber')
    velodyne_subscriber = rospy.Subscriber('/lidar1/velodyne_points', PointCloud2, lidar_callback)
    steer_subscriber = rospy.Subscriber('/pacmod/parsed_tx/steer_rpt', SystemRptFloat, steer_callback)
    speed_subscriber = rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, speed_callback)

    thread = Thread(target=render)
    thread.start()

    desired_speed = 0.3
    move_forward(desired_speed)

    rospy.spin()
    kill_thread = 1
    thread.join()
    print( 'success' )

if __name__=='__main__':
    montecarlo_demo()
