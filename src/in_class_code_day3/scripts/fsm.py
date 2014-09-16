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
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

distance_to_wall = -1
target = 1.0

def scan_received(msg):
    global distance_to_wall
    if len(msg.ranges) != 360:
        print 'unexpcted laser scan message'
        return

    valid_msgs = 0.0
    sum_valid = 0.0
    for i in range(5):
        if msg.ranges[i] > 0.1 and msg.ranges[i] < 7.0:
            valid_msgs += 1
            sum_valid += msg.ranges[i]
            print msg.ranges[i]
    if valid_msgs > 0:
        distance_to_wall = sum_valid / valid_msgs
    else:
        distance_to_wall = -1

def getch():
    """ Return the next character typed on the keyboard """
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def teleop(pub):
    turn_vel = .4
    linear_vel = 1
    r = rospy.Rate(10)

    while not(rospy.is_shutdown()):
        if distance_to_wall < 0.5 and distance_to_wall > 0:
            return "approach_wall"
        c = getch()
        if c == 'i':
            pub.publish(Twist(linear=Vector3(x=linear_vel)))
        elif c == 'u':
            pub.publish(Twist(linear=Vector3(x=linear_vel),
                              angular=Vector3(z=turn_vel)))
        elif c == 'o':
            pub.publish(Twist(linear=Vector3(x=linear_vel),
                              angular=Vector3(z=-turn_vel)))
        elif c == 'j':
            pub.publish(Twist(angular=Vector3(z=turn_vel)))
        elif c == 'l':
            pub.publish(Twist(angular=Vector3(z=-turn_vel)))
        elif c == 'm':
            pub.publish(Twist(linear=Vector3(x=-linear_vel),
                              angular=Vector3(z=-turn_vel)))
        elif c == ',':
            pub.publish(Twist(linear=Vector3(x=-linear_vel)))
        elif c == '.':         
            pub.publish(Twist(linear=Vector3(x=-linear_vel),
                              angular=Vector3(z=turn_vel)))
        elif c == 'q':
            break
        else:
            pub.publish(Twist())
        print distance_to_wall
        r.sleep()

def approach_wall(pub):
    r = rospy.Rate(10)
    while not(rospy.is_shutdown()):
        if distance_to_wall != -1:
            pub.publish(Twist(linear=Vector3(x=.4*(distance_to_wall-target))))
        r.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('my_fsm', anonymous=True)
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        sub = rospy.Subscriber('scan', LaserScan, scan_received)
        state = "teleop"

        while not rospy.is_shutdown():
            if state == 'teleop':
                state = teleop(pub)
            elif state == 'approach_wall':
                state = approach_wall(pub)
    except rospy.ROSInterruptException: pass
