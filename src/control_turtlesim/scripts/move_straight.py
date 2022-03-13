#!/usr/bin/env python
from turtle import pos
import rospy
from geometry_msgs.msg import Twist # For PUB
from turtlesim.msg import Pose      # For SUB
import math
import time

# is_forward: True > forward, False > Go back
def move(velocity_publisher, speed, distance, is_forward):

    velocity_message = Twist()

    # Get curent location
    global x, y
    x0 = x # Init location x
    y0 = y # Init location y

    if (is_forward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10) # 10Hz Using for sleep

    # Send move message
    while True:
        rospy.loginfo("Turtlesim moves")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        distance_moved = abs( math.sqrt( (x-x0)**2 + (y-y0)**2 ) )
        print(distance_moved)

        if not (distance_moved < distance):
            rospy.loginfo("Reached")
            break

    # Stop the robot when the distance is moved
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)

# Sub callback func, get location and set global x, y, yaw
def poseCallback(pose_message):
    global x
    global y, yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta

if __name__ == '__main__':

    try:
        # Init node
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        # Declare Pub
        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Declare Sub
        pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, poseCallback)
        time.sleep(2)

        move(velocity_publisher, 1.0, 4.0, True)
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")


# Confirm command
# rostopic list
# rostopic info /turtle1/cmd_vel
# rosmsg show geometry_msgs/Twist