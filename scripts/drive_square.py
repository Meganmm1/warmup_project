#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class BigSquare(object):
    
    def __init__(self):
        #insitializes a ros node and creates a publisher 
        rospy.init_node('velocity')
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

   

    def linear(self):
        #makes the robot move in a straight line
        initial_linear_velocity = Vector3(0.3,0.0,0.0)
        initial_angular_velocity = Vector3(0.0,0.0,0.0)
        my_twist = Twist(linear = initial_linear_velocity, angular = initial_angular_velocity)
        r = rospy.Rate(2)
        #the publisher rate which is 2hz
        for i in range(2):  
            #At a rate of 2 hz the robot should move in a straight line for 1 second
            while not rospy.is_shutdown():
                #allows the publiher to publish multiple messages at a time.
                rospy.sleep(1)
                self.twist_pub.publish(my_twist)
                rospy.sleep(2)
                break
    
    #rostopic echo and topic name
    def turn(self):
        #makes the robot move at a 90 degree angle
        my_linear_velocity = Vector3(0.0,0.0,0.0)
        my_angular_velocity = Vector3(0.0,0.0,0.25)
        my_twist = Twist(linear = my_linear_velocity, angular = my_angular_velocity)
        r = rospy.Rate(2)
        #the publisher rate which is 2hz
        for i in range(2):
            #Robot should turn for a total of 1 seconds
            while not rospy.is_shutdown():
                rospy.sleep(2)
                self.twist_pub.publish(my_twist)
                rospy.sleep(2)
                break

    def run(self):
        for i in range(4):
            #loops through the line and turn 4 times
            node.linear()
            node.turn()
        

if __name__ == '__main__':
    node = BigSquare()
    node.run()
