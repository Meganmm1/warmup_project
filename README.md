# warmup_project

drive_square.py write up
    1. In this portion of the project I intended on making my robot go in a straight line and then turn, both happening for a given period of time. In order to make my robot trace out a square I first created a function that defined the linear motion of the robot. The idea was to get my robot to move foward for about 1 second with no angular velocity and then stop. I then defined a function for the angular motion of the robot which is to be executed after the linear function for about second1 and then stop. Over 4 iterations of these two functions my robot would move and turn to make a square.
    2. My first function is called linear and is part of the Big square class. I initialized a linear velocity and an angular velocity where my angular velocity is set to zero. I then set up a for loop that iterates through the publish call. I added a sleep call before the publisher to account for lag and then a sleep function after to allow for my next function to run. 
    My second function is called turn also part of the Big square function where I used the same methology as my linear function but instead I kept the angular velocity as nonzero. 
    My last function is the run function which calls my frist two functions and I do this in a four loop so that it repeats 4 times to make a square.

