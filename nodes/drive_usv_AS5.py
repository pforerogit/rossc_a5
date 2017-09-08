#!/usr/bin/env python

# System imports
import sys
import time
from math import *
import numpy as np
# Rospy import
import rospy

# Import message and service types
from nav_msgs.msg import Odometry
from kingfisher_msgs.msg import Drive
from geometry_msgs.msg import Twist
#from turtlesim.msg import Pose

widx = 0

def wcontrol(px,py,ox,oy,wp_x,wp_y):
    ''' 
    Function to implement waypoint guidance
    % Inputs:
    %  - x: Current X position
    %  - y: Current Y position
    %  - th: Current theta position
    %  - wp_x: Current waypoint goals, X
    %  - wp_y: Current waypoint goals, X
    % Outputs (as a tuple)
    %  - dist: Distance from current position to current waypoint goal
    %  - angvel: Desired angular velocity [rad/s, clockwise]
    %  - linvel: Desired linear velocity 
    '''

    # INSERT CALCULATIONS HERE
    dist = np.sqrt((px-wp_x)**2+(py-wp_y))
    angvel = 0.0
    linvel = 0.0

    # Return a tuple
    return (dist,angvel,linvel)

def callback_fcn(msg,args):
    ''' 
    This function gets called whenever we receive a new Pose message
    '''
    global widx
    # Process the callback arguments
    cmdPub = args[0]  # the publisher
    cmdMsg = args[1]  # our Twist message
    waypoints = args[2]  # List of waypoints
    #widx = args[3]  # Current waypoint index
    #distThreshold = args[4]  # Distance threshold

    # Get x, y and theta from pose message
    px = msg.pose.pose.position.x
    py = msg.pose.pose.position.y
    pz = msg.pose.pose.position.z
    ox = msg.pose.pose.orientation.x
    oy = msg.pose.pose.orientation.y
    oz = msg.pose.pose.orientation.z
    ow = msg.pose.pose.orientation.w
    print('Received a Pose position message: X:%f, Y:%f, Z:%f'%(px,py,pz))
    print('Received a Pose orientation message: X:%f, Y:%f, Z:%f, W:%f'%(ox,oy,oz,ow))
    # Call the waypoint control algorithm
    if widx < len(waypoints):
    	wp_x = waypoints[widx][0]
    	wp_y = waypoints[widx][1]
    	dist, angvel, linvel = wcontrol(px,py,ox,oy,wp_x,wp_y)
    print('WIDXNEW = ' + str(widx)+ '    Waypontslen = ' +str(len(waypoints)))
    # Evaluate what to do next based on the distance to the waypoint.
    if widx >= len(waypoints):
	print('done!')
	rospy.signal_shutdown("Finished")
#    if (dist <= 1.0):
#        if (widx < len(waypoints)-1):
#            print('Going to next waypoint!')
#            args[3] = widx+1  # Need to increment the actual argument 
#        else:
#            print('Done!')
#            rospy.signal_shutdown("Finished")
#    else:
        # Populate the twist message 
#        print('WP %d at x: %f, y: %f'%(widx, wp_x,wp_y))
#        cmdMsg.linear.x = linvel
#        cmdMsg.angular.z = angvel
#        print('Publishing cmd_vel with lin. vel: %f, ang. vel.: %f'
#              %(linvel,angvel))
#        cmdPub.publish(cmdMsg)
    cmdMsg.left = wp_x
    cmdMsg.right = wp_y
    cmdPub.publish(cmdMsg)
    widx = widx + 1	
##########  Start of control script ##########
# Initialize the node and wait a moment
print('Initializing USV Control ROS node')
rospy.init_node('Pedro_USV_python_example')
time.sleep(1)

# List of waypoints (x,y) in order of operation
waypoints = [[1,-1],
             [1,1],
             [1,1],
	     [1,1],
             [1,1],
             [1,1],
             [1,1],
             [0.3, -0.3],
	     [0.3, -0.3],
	     [1,1],
             [1,1],
             [1,1],
             [1,1],
             [1,1],
             [1,1],
             [1,1],
             [1,1]];
#widx = 0;  # Index of active waypoint

# Set a distance metric for reaching a waypoint
distThreshold = 3.0;

# Setup publication of Twist commands
### NEEDS TO CHANGE
cmdPub = rospy.Publisher('/cmd_drive',Drive,queue_size=10);
time.sleep(2); # Wait to ensure publisher is registered

for i in range(len(waypoints)):
    # Create an empty Twist message for publication
    cmdMsg = Drive()
    cmdMsg.left = waypoints[i][0]
    cmdMsg.right = waypoints[i][1]
    cmdPub.publish(cmdMsg)
    time.sleep(3)

# Setup subscriber - pointing it to the callback function above
# Note we use the callback_args to pass the publisher and message into
# the callback function (Could have used globals, but...)
#poseSub = rospy.Subscriber('/p3d_odom',Odometry,
#                           callback=callback_fcn,
#                           callback_args=[cmdPub,cmdMsg,
#                                          waypoints])
#print('WIDX = ' + str(widx))
# This just waits until is_shutdown() becomes true - infinite loop
# See http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
#rospy.spin()

