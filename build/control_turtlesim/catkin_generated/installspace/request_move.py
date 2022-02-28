#!/usr/bin/env python3
import rospy

# from std_msgs.msg import String
# For PUB
from geometry_msgs.msg import Twist
# For SUB
from turtlesim.msg import Pose

def chatter_callback(message):
    show_msg = "\nx: {}\ny: {}\ntheta: {}\nv_liner: {}\nv_angular: {}".format(
        message.x, message.y, message.theta, message.linear_velocity, message.angular_velocity)
    rospy.loginfo(rospy.get_caller_id() + show_msg)

def move():
    # Create a new publisher
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz loop rate

    # Subscriber for the topic that will show the location of the robot
    rospy.Subscriber("/turtle1/pose", Pose, chatter_callback)

    while not rospy.is_shutdown():
        # Publisher to the topic that will make the robot move
        twist = Twist()
        twist.angular.x = 0.1
        twist.linear.x = 0.1
        twist.linear.y = 0.1
        pub.publish(twist)

        rate.sleep()

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
