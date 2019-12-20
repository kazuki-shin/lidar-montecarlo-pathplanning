#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import Bool
from pacmod_msgs.msg import PacmodCmd
import time

# pacmod_msgs/PacmodCmd


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
    enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=10)
    brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd',
                                PacmodCmd,
                                queue_size=10)
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
