#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

#How close the robot gets to the wall
min_distance = 0.3

class WallFollower(object):
    "This node allows the robot to follow along the walls of a room"
    def __init__(self):
        #start rospy node
        rospy.init_node("follow_wall")

        #declares node as a subscriber to scan topic and makes a callback
        # function named interpret_scan
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        #sets a publisher for the cmd_vel topic
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        #initializes an all zero Twist message
        linear= Vector3()
        angular= Vector3()
        self.twist= Twist(linear= linear, angular= angular)

    def process_scan(self,data):
        i=0
        kp = .01
        shortest_distance = 80
    
        for i in range (360):
            #Will loop through all the angles and store the angle at which 
            #the closest object to the robot is located
            if data.ranges[i] < shortest_distance and not data.ranges[i]==0:
                shortest_distance = data.ranges[i]
                min_i = i

        if shortest_distance == 80:
            #if no object is near then the robot will slowly move foward
            self.twist.linear.x = .05
            self.twist.angular.z= 0
        
        if data.ranges[min_i] >= min_distance:
            #if the robot is farther than the fixed distance move it foward 
            #proportional control ensure that the closest object is at an angle of 0
            # in which case object would not turn if facing the wall
            self.twist.linear.x = .1
            self.twist.angular.z = kp*(min_i)
            
        
        if min_i < 10 or min_i > 350 : 
            #if there is an object directly in front of the robot
            # it will move foward and turn in realations to the nearest
            #object being at 270 degrees 
            self.twist.linear.x = .1
            self.twist.angular.z = kp*(270-min_i)
            
        
        if min_i > 265  or min_i < 275:
            #if the nearest object is located at an angle of 270 degree which is
            #a fixed distance from the wall then the robot will just move foward
            self.twist.linear.x = .1
            self.twist.angular.z= 0
            
        
       
        if min_i <= 265 or min_i >= 10 :
            #if the robot is tilted right from the wall then it will move foward and
            #turn left so that the nearest object is located at 270 degrees
            self.twist.linear.x= .1
            self.twist.angular.z = - kp*(270-min_i)
            
        
        if min_i <= 350 or min_i >= 275 :
            #if the robot is tilted left from the wall then it will move foward and
            #turn right so that the nearest object is located at 270 degrees
            self.twist.linear.x= .1
            self.twist.angular.z = - kp*(270-min_i)
           
        
        if data.ranges[min_i] < min_distance :
            #if the robot is closer than the fixed distance this condition
            #is meant to move the robot away from the wall.
            self.twist.linear.x = kp *(min_distance- data.ranges[min_i])
            self.twist.angular.z = kp *(min_i)
           
 
        self.twist_pub.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__== '__main__':
    node = WallFollower()
    node.run()