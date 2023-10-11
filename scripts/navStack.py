#!/usr/bin/python3

# Team ID:		    [ KB_1492 ]
# Author List:		[ vidya vepoori , ch pranathi ]
# Filename:		    navStack.py
# Functions:		clbk_laser(msg),navagation(),nav_left_wall(),nav_right_wall(),
#                   forward(),stop(),control_loop()
# Nodes:	        pub to : /cmd_vel ,sub to : /ebot/laser/scan


# import ros stuff

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# inizalizing global variables

pub = None

# defining dist correction 
l_correction=0.0
r_correction=0.0

# defining desired dist and current dist
l_dist_desired=0.71
r_dist_desired=2.3
l_dist=0.0
r_dist=0.0

# defining kp constant
kp=0.35

# defining vel and angle variables
vel=0.2
n=0.0

# sim time inti=0.0
curr_time=0.0

def clbk_laser(msg):
    global l_correction,r_correction,l_dist_desired,r_dist_desired,l_dist,r_dist

    regions = {
    'right': min((msg.ranges[3]), 10),
    'fright': min((msg.ranges[278]), 10),
    'front': min(min(msg.ranges[288:431]), 10),
    'fleft': min((msg.ranges[640]), 10),
    'left': min((msg.ranges[719]), 10),
    }

    # left and fleft values 
    a=regions['fleft']
    b=regions['left']
    c=regions['fright']
    d=regions['right']

    # calculating ebot distance from left wall
    l_dist=(a+b)/2
    # finding angle to correct the ebot direction to follow left wall
    l_correction= abs(l_dist_desired - l_dist )

    # calculating ebot distance from right wall
    r_dist=(c+d)/2
    # finding angle to correct the ebot direction to follow right wall
    r_correction= abs(r_dist_desired - r_dist ) 

    # stating navagation
    navagation()

def navagation():

    #  move forward till 2sec
    if curr_time<2:
        forward()

    # follow left wall    
    if curr_time>=2 :  
        nav_left_wall()    


# to follow left wall   
def nav_left_wall():
    
    global vel_cmd,vel,n,curr_time
    global l_correction,l_dist_desired,l_dist

    # limits for angle crrection
    minn=0.1
    maxn=10.5
    n=kp*l_correction
    n= minn if n < minn else maxn if n > maxn else n

    print('angel',n,'\n l_dist',l_dist)

    if l_dist>l_dist_desired : 
        print(' taking turn left ')
        vel_cmd.linear.x=vel
        vel_cmd.angular.z= n
        #print('left z',vel_cmd.angular.z)
        pub.publish(vel_cmd)

    if l_dist<l_dist_desired : 
        print(' taking right turn ')
        vel_cmd.linear.x=vel
        vel_cmd.angular.z= -n
        #print('right z',vel_cmd.angular.z)   
        pub.publish(vel_cmd)
   
# to follow right wall
def nav_right_wall():  

    global vel_cmd,vel,n,curr_time
    global r_correction,r_dist_desired,r_dist

    # limits for angle correction
    minn=0.1
    maxn=10.5 #10.5
    n=kp*r_correction
    n= minn if n < minn else maxn if n > maxn else n

    print('angel',n,'\n r_dist',r_dist)

    if r_dist > r_dist_desired : 
        print(' taking right turn ')
        vel_cmd.linear.x=vel
        vel_cmd.angular.z=-n
        print('right z',vel_cmd.angular.z)
        pub.publish(vel_cmd)

    if r_dist < r_dist_desired : 
        print(' taking left turn ')
        vel_cmd.linear.x=vel
        vel_cmd.angular.z= n
        print('left z',vel_cmd.angular.z)   
        pub.publish(vel_cmd)

# additional motor directions 
               
def forward():
    print(' forward ')
    vel_cmd.linear.x=0.7
    vel_cmd.angular.z= 0.0
    pub.publish(vel_cmd)

def stop():
    print(' closed ')
    vel_cmd.linear.x=0.0
    vel_cmd.angular.z= 0.0
    pub.publish(vel_cmd)
           
# control loop for running the code fro navagation           
def control_loop():

    global pub, vel_cmd,curr_time

    rospy.init_node('ebot_controller')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('ebot/laser/scan',LaserScan,clbk_laser)
    vel_cmd = Twist()
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():

        #getting sim time convert to sec 
        now = rospy.Time.now()
        curr_time=now.secs
        print('current_time',curr_time)
       
        rate.sleep()

# main loop and handles any exceptions 
if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
