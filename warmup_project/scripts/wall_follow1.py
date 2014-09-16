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
import math
import thread, time
import sys, select, termios, tty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

lastKey = 'hi'
scan = [-1] * 360
targetDist = .75
hasWall = False

moveBindings = {
		'w':(1,0),
		's':(-1,0),
		'a':(0,1),
		'd':(0,-1),
	       }

def keyboardCommand():
    pub = rospy.Publisher('cmd_vel', Twist)
    sub = rospy.Subscriber('scan', LaserScan, scan_received)
    rospy.init_node('maci_neato', anonymous=True)
    r = rospy.Rate(10) # 60hz
    currentTask = 'keyboardControl'
    thread.start_new_thread(input_thread, (lastKey,))
    while not rospy.is_shutdown():
        tempLastKey = lastKey
        
        if tempLastKey in moveBindings.keys():
            print tempLastKey
            if currentTask != 'keyboardControl':
                print 'keyboardControl'
            currentTask = 'keyboardControl'
            keyboardControl(pub, tempLastKey)

        elif (tempLastKey == 'f'):
            if currentTask != 'wallFollow':
                print 'wallFollow'
            currentTask = 'wallFollow'
            wallFollow(pub)
            
    	elif (tempLastKey == '\x03'):
            stopRobot()
            break
        else:
            if currentTask != 'stop':
                    print 'stop'
            currentTask = 'stop'
            sendCommand(pub,0,0)
        r.sleep()

def wallFollow(pub):
    tempScan = scan
    dist45 = averagePoint(65,tempScan)
    dist135 = averagePoint(115,tempScan)

    # if dist45 == -1:
    #     dist45 = 7
    # if dist135 == -1:
    #     dist135 = 7

    # normDist45 = dist45/(dist45 + dist135)
    # normDist135 = dist135/(dist45 + dist135)
    # turn = normDist135/normDist45 * 10

    wall = findClosestWall(tempScan)
    if wall == 'na':
        print 'no wall detected'
        return
    angFromWall = 90 - wall

    distFromWall = averagePoint(wall,tempScan)

    # for i in range(45,135):
    #     dist = averagePoint(i,tempScan)
    #     if dist != -1:
    #         distFromWall = dist

    # print "dist45: " + str(dist45)
    # print "dist135: " + str(dist135)
    print "angFromWall: "+ str(angFromWall)
    print "distFromWall: "+ str(distFromWall)
    angFromAng = -1*angFromWall/100.0
    angFromDist = ((1/(abs(angFromWall/2.0+.01)))**1.5)*(distFromWall-targetDist)*10 #(1+abs(targetDist-distFromWall))*



    print "angFromAng: " + str(angFromAng)
    print "angFromDist: " + str(angFromDist)
    if abs(targetDist-distFromWall)<.05:
        angFromDist2 = 0
    ang = angFromAng + angFromDist
    print "ang: " + str(ang)
    # if ang>.4:
    #     ang = .4
    # elif ang< (-.4):
    #     ang = (-.4)
    sendCommand(pub, .1, ang)

def findClosestWall(tempScan):
    angBetweenValuesArray = [90,80,70,60,50,44,38,26,16]

    possibleWalls = [0] * 360

    for j in range(len(angBetweenValuesArray)):
        minDistToWall1 = 100
        angBetweenValues = angBetweenValuesArray[j] #must be even
        for i in range(65,115):
            lowerAng = ((i - angBetweenValues/2) +360)%360
            upperAng = ((i + angBetweenValues/2) +360)%360
            lowerAngDist = averagePoint(lowerAng,tempScan)
            upperAngDist = averagePoint(upperAng,tempScan)
            dif = abs(lowerAngDist-upperAngDist)
            if dif < .1 and averagePoint(i,tempScan) < minDistToWall1:
                possibleWalls[i] += 1

    averageVotes = [0] * 360
    for k in range(360):
        averageVotes[k] = averagePoint(k,possibleWalls,numPoints=20)

    return averageVotes.index(max(averageVotes))



def averagePoint(scanR,tempScan,numPoints=5):
    sideRange = (numPoints-1)/2
    valid_msgs = 0
    sum_valid = 0.0
    for i in range(scanR-numPoints,scanR+numPoints):
        j = i%360
        if tempScan[j] > 0.1 and tempScan[j] < 7.0:
            valid_msgs += 1
            sum_valid += tempScan[j]
    if valid_msgs > 0:
        return sum_valid / valid_msgs
    else:
        return -1

def scan_received(msg):
    """ Callback function for msg of type sensor_msgs/LaserScan """
    global scan
    global distance_to_wall
    if len(msg.ranges) != 360:
        print 'unexpcted laser scan message'
        return

    for i in range(360):
        if msg.ranges[i] > 0.1 and msg.ranges[i] < 7.0:
            scan[i] = msg.ranges[i]
        else:
            scan[i] = -1

def keyboardControl(pub, lastKey):
    lin = moveBindings[lastKey][0]
    ang = moveBindings[lastKey][1]
    sendCommand(pub,lin,ang)

def stopRobot():
    pub = rospy.Publisher('cmd_vel', Twist)
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)

def sendCommand(pub, lin, ang):
	twist = Twist()
	twist.linear.x = lin; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = ang
	pub.publish(twist)

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

def input_thread(lastKey1):
    #raw_input()
    while not rospy.is_shutdown():
        global lastKey
        lastKey = getch()
        if lastKey == '\x03':
            break
        
if __name__ == '__main__':
    try:
        keyboardCommand()
    except rospy.ROSInterruptException: pass