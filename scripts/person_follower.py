#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

#How close the robot gets to a person
min_distance = 0.4

class FollowPerson (object):
    """This node allows the robot to follow a person"""

    def __init__(self):
        #start rospy node
        rospy.init_node("follow_person")

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
            #if no object is near then the robot will not move
            self.twist.linear.x = 0
            self.twist.angular.z= 0
        
        if min_i < 10 or min_i > 350:
            #will move the robot foward if there is an object directly infront of it
            #ask about why it doesnt back up when i stand directly in front
            self.twist.linear.x = .1
            self.twist.angular.z= 0

        elif data.ranges[min_i] > min_distance :
            #will detect if there is an object greater than .3m 
            #and move the robot closer while turning the robot 
            #using proportional control trying to keep the closet object at a 
            #zero degree angle. 
            self.twist.linear.x = .1
            self.twist.angular.z= kp*(min_i)

        elif data.ranges[min_i] < min_distance:
            #will back up the robot if an object gets closer than .3m 
            self.twist.linear.x= -.2
            self.twist.angular.z= kp*(min_i)
        
        elif data.ranged[min_i] == min_distance:
            #will stop the robot if they are exactly .4m away from a person
            self.twist.linear.x =0
            self.twist.angular.z = 0
            
        self.twist_pub.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__== '__main__':
    node = FollowPerson()
    node.run()

                 

