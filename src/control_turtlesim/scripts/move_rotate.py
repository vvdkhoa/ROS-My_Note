#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist # For PUB
from turtlesim.msg import Pose      # For SUB
import math
import time

def rotate (velocity_publisher, angular_speed_degree, relative_angle_degree, clockwise):

    # Create message obj
    velocity_message = Twist()

    angular_speed = math.radians(abs(angular_speed_degree))
    if (clockwise):
        velocity_message.angular.z = -abs(angular_speed)
    else:
        velocity_message.angular.z = abs(angular_speed)
    
    # Setting Hz
    loop_rate = rospy.Rate(10)  # 10 Hz
    t0 = rospy.Time.now().to_sec()

    # Send message request move
    while True:
        rospy.loginfo("Turtlesim rotates")

        # Publish
        velocity_publisher.publish(velocity_message)

        # Caculate current angle = time x speed
        t1 = rospy.Time.now().to_time()
        current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep()

        if current_angle_degree > relative_angle_degree:
            rospy.loginfo("Reached")
            break
    
    # Stop when moved
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)


# Sub callback
def poseCallback(pose_message):
    global yaw
    yaw = pose_message.theta
    rospy.loginfo("Theta: {}".format(yaw))


if __name__ == '__main__':
    
    try:
        # Init node
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        # Declare Pub
        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Declare Sub
        pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, poseCallback)
        time.sleep(2)

        # Rotate robot
        rotate(velocity_publisher, 1.0, 90, True)
    
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")
