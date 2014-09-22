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


def teleop():
    pub = rospy.Publisher('/r2d2/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/johnnyfive/cmd_vel', Twist, queue_size=10)

    rospy.init_node('my_keyboard_teleop', anonymous=True)
    turn_vel = .4
    linear_vel = 1
    while not rospy.is_shutdown():
        c = getch()
        if c == 'i':
            m = Twist(linear=Vector3(x=linear_vel))
        elif c == 'u':
            m = Twist(linear=Vector3(x=linear_vel),
                      angular=Vector3(z=turn_vel))
        elif c == 'o':
            m = Twist(linear=Vector3(x=linear_vel),
                      angular=Vector3(z=-turn_vel))
        elif c == 'j':
            m = Twist(angular=Vector3(z=turn_vel))
        elif c == 'l':
            m = Twist(angular=Vector3(z=-turn_vel))
        elif c == 'm':
            m = Twist(linear=Vector3(x=-linear_vel),
                              angular=Vector3(z=-turn_vel))
        elif c == ',':
            m = Twist(linear=Vector3(x=-linear_vel))
        elif c == '.':         
            m = Twist(linear=Vector3(x=-linear_vel),
                              angular=Vector3(z=turn_vel))
        elif c == 'q':
            break
        else:
            m = Twist()

        pub.publish(m)
        pub2.publish(m)
        
if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException: pass
