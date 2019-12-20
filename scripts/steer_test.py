#!/usr/bin/env python
# min turning diameter of 13ft11in
# max steering wheel agle of +-3.5pi
# 2.13m turning radius
# wheelbase = 1.676 m
import rospy
from std_msgs.msg import Bool
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, SteeringPIDRpt3
import time
import numpy as np
import argparse
import time
import math

# pacmod_msgs/PacmodCmd

enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=10)
steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=10)
msg = PositionWithSpeed()
msg.angular_position = (np.pi/6)
msg.angular_velocity_limit = (np.pi/2)/2

test_steer_pub = rospy.Publisher('parsed_tx/steer_pid_rpt_3', SteeringPIDRpt3, queue_size=10)

S = 1
T = 1
W = 1.676

#position
# heading in radians to straight vertical
# speed
# timestep
# angle of wheels
def forward(pos,heading,s,t,angle):
    # becase we modeled with positive being clockwise,
    # but the car has negative as clockwise
    angle = -angle
    rot = np.array([[math.cos(heading),-math.sin(heading)],[math.sin(heading),math.cos(heading)]])
    if angle == 0:
        displacement = np.array([0,s*t])
        theta = 0
    elif angle > 0:
        r = W/math.sin(angle)
        # distance moved around circle in radians
        theta = s*t/r
        # rotation matrix from heading
        displacement = np.array([math.cos(theta)-1,math.sin(theta)])*r
    else:
        r = W/math.sin(-angle)
        theta = s*t/r
        displacement = np.array([-(math.cos(theta)-1),math.sin(theta)])*r
    new_pos = np.matmul(rot,displacement) + pos
    return new_pos, math.fmod(heading + theta,np.pi*2)

pos = np.array([0,0])
heading = np.pi
print(pos,heading)
pos,heading = forward(pos,heading,S,T,np.pi/100)

controls = []
def sendSteer(ang,speed = (np.pi/2)/2):
    enable = Bool()
    enable.data = True
    enable_pub.publish(enable)


    msg = PositionWithSpeed()
    msg.angular_position = ang
    msg.angular_velocity_limit = speed
    steer_pub.publish(msg)
    controls.append((time.time(),ang,speed))
    np.save("controls",np.stack(controls))

frames = []
def listening(msg):
    print(msg)
    frames.append((time.time(),msg.str_angle_actual))
    np.save("turn_data",np.stack(frames))

def steer():
    rospy.init_node('steering', anonymous=True)
    import pdb; pdb.set_trace()
    # test_steer_pub = rospy.Publisher('parsed_tx/steer_pid_rpt_3', SteeringPIDRpt3, queue_size=10)
    # msg = SteeringPIDRpt3()
    # msg.str_angle_actual = 100
    # test_steer_pub.publish(msg)
    sendSteer(np.pi/6)


def listener():
    print("Launching listener")
    rospy.init_node('listening', anonymous=True)
    rospy.Subscriber("parsed_tx/steer_pid_rpt_3", SteeringPIDRpt3, listening)
    rospy.spin()
    # rate = rospy.Rate(0.3)
    # while not rospy.is_shutdown():
        # rospy.loginfo("brake at %s" % rospy.get_time())
        # sendBrake(enable_pub, brake_pub)
        # time.sleep(1)
        # rospy.loginfo("un brake at %s" % rospy.get_time())
        # sendNoBrake(enable_pub, brake_pub)
        # rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', dest='listen', action='store_true')
    args = parser.parse_args()
    if args.listen:
        listener()
    else:
        steer()
    # try:
        # talker()
    # except rospy.ROSInterruptException:
        # pass
