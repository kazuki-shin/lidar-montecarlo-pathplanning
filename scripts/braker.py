#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from pacmod_msgs.msg import PacmodCmd
import time

# pacmod_msgs/PacmodCmd

enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=10)
brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd',
                            PacmodCmd,
                            queue_size=10)

# topic
# as_rx/brake_cmd
def sendBrake(enable_pub, brake_pub):
    enable = Bool()
    enable.data = True
    enable_pub.publish(enable)

    brake = PacmodCmd()
    brake.enable = True
    brake.f64_cmd = 1.0
    brake_pub.publish(brake)


def sendNoBrake(enable_pub, brake_pub):
    enable = Bool()
    enable.data = True
    enable_pub.publish(enable)

    brake = PacmodCmd()
    brake.enable = True
    brake.f64_cmd = 0.0
    brake_pub.publish(brake)


def talker():
    rospy.init_node('braker', anonymous=True)
    rate = rospy.Rate(0.3)  # every 10s
    while not rospy.is_shutdown():
        rospy.loginfo("brake at %s" % rospy.get_time())
        sendBrake(enable_pub, brake_pub)
        time.sleep(1)
        rospy.loginfo("un brake at %s" % rospy.get_time())
        sendNoBrake(enable_pub, brake_pub)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
