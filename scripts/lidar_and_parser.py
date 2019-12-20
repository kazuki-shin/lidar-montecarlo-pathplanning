'''
    Input from LIDAR
'''

import sys
package_path='/site-packages35'                       # -- change this path
sys.path.insert(0,package_path)

import os
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from pacmod_msgs.msg import SteeringPIDRpt3,SteeringPIDRpt1,SystemRptFloat

from std_msgs.msg import String, Bool, Float32, Float64
from steer_test import sendSteer
import numpy as np
import time
from matplotlib import pyplot as plt
from threading import Lock, Thread
from plan import T,turn_rad_sec,max_steer_angle,clip

from move import move_forward

frame_lock = Lock()

DUMMY_FIELD_PREFIX = '__'

# mappings between PointField types and numpy types
type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]

pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

# sizes (in bytes) of PointField types
pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}

def fields_to_dtype(fields, point_step):
    '''
    Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_sizes[f.datatype] * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list

def msg_to_arr(msg):

    dtype_list = fields_to_dtype(msg.fields, msg.point_step)
    arr = np.fromstring(msg.data, dtype_list)

    # remove the dummy fields that were added
    arr = arr[[fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]

    if msg.height == 1:
        return np.reshape(arr, (msg.width,))
    else:
        return np.reshape(arr, (msg.height, msg.width))

##################################################################################################################################

lidar_dir = './data'                                   # -- change this path

def callback(msg):
    global latest_frame
    timestamp = msg.header.stamp.to_nsec()

    # print('callback: msg : seq=%d, timestamp=%19d'%( msg.header.seq, timestamp))
    arr= msg_to_arr(msg)

    '''
        Parse input file
    '''

    xs = [row[0] for row in arr]
    ys = [row[1] for row in arr]
    zs = [row[2] for row in arr]
    mat = np.asarray([xs,ys,zs]).T

    # filtering - currently hardcoded
    mat = mat[(mat[:,2]<0) & (mat[:,2]>-1.5)] # z-axis
    mask = (mat[:,0] < 1.5) & (mat[:,0]>-1.5) & (mat[:,1] < 1) & (mat[:,1] > -1) # x-axis
    mat = mat[np.invert(mask)]

    #mat = mat[(mat[:,1]>=1) | (mat[:,1]<=-1)] # y-axis

    latest_frame = mat[:,[0,1]]

    # from plan import display_plan
    # import cv2
    # img = display_plan(mat[:,[0,1]])
    # cv2.imshow('plan',img)
    # cv2.waitKey(1)

    # 2D for planning
    # grid = np.column_stack((mat[:,0],mat[:,1]))

    # fig = plt.figure()
    # plt.scatter(mat[:,0],mat[:,1])
    # plt.show()

    # return grid

def steer_callback(msg):
    global current_steer_angle
    current_steer_angle = msg.output
    #print("current_steer_angle: "+str(current_steer_angle))

from threading import Lock, Thread
import cv2
from plan import display_plan
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
            # testing using the speed from the controller
            # img,node = display_plan(latest_frame,current_steer_angle,current_speed)
	    next_node = node.best_rollout[1]
            # diff = max(min(next_node.control_value,max_change),-max_change)
            # diff = clip(angle,-max_angle_change,max_angle_change)
            diff = next_node.control_value
            
            next_step = diff + current_steer_angle
            next_step = clip(next_step,-max_steer_angle,max_steer_angle)
            # print("CURRENT_ANGLE: "+str(current_steer_angle))
            print("Plan Time: ",time.time() - start)
            start = time.time()
            sendSteer(next_step,turn_rad_sec+1)
            cv2.imshow('plan',img)
            cv2.waitKey(1)

def speed_callback(msg):
    #print(msg)
    current_speed = msg.data



if __name__=='__main__':
    print( '%s: calling main function ... ' % os.path.basename(__file__))

    if not os.path.exists(lidar_dir):
        os.makedirs(lidar_dir)

    rospy.init_node('velodyne_subscriber')
    velodyne_subscriber = rospy.Subscriber('/lidar1/velodyne_points', PointCloud2, callback)
    steer_subscriber = rospy.Subscriber('/pacmod/parsed_tx/steer_rpt', SystemRptFloat, steer_callback)
    speed_subscriber = rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, speed_callback)
    
    thread = Thread(target=render)
    thread.start()
    
    desired_speed = 0.3
    move_forward(desired_speed)

    # time.sleep(5)
    rospy.spin()
    kill_thread = 1
    thread.join()
    print( 'success' )
