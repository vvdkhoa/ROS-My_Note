#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

# Document: http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal

class TurtleBot():

    def __init__(self):

        # Init unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Create Pub
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Create Sub
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

        # Setting Hz
        self.rate = rospy.Rate(10)

        self.pose = Pose()
    
    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def eulidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal.
            Khoang cach nho nhat giua 2 diem"""
        return sqrt( pow( (goal_pose.x - self.pose.x), 2 )
                    + pow( (goal_pose.y - self.pose.y), 2 ) )

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.eulidean_distance(goal_pose)
    
    def angular_vel(self, goal_pose, constant=6):
        steering_angle = atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
        return constant * (steering_angle -self.pose.theta)
        
    def move_to_goal(self, x_goal=None, y_goal=None, distance_tolerance=None):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get input from cmd when using move_to_goal()
        if not (x_goal and y_goal and distance_tolerance):
            goal_pose.x = float( input("Set your x goal: ") )
            goal_pose.y = float( input("Set your y goal: ") )
            distance_tolerance = float( input("Set you torelance: ") )
        # When using  move_to_goal (x_goal, y_goal, distance_tolerance), get param from launch file
        else:
            goal_pose.x = x_goal
            goal_pose.y = y_goal
            distance_tolerance = distance_tolerance

        vel_msg = Twist()

        while self.eulidean_distance(goal_pose) >= distance_tolerance:
            
            # Porportional controller.
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Pub
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()
        
        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':

    try:

        bot = TurtleBot()

        # Way 1: Get param from cmd
        # bot.move_to_goal()

        # Way 2: Get param in launch file
        x_goal = rospy.get_param("x_goal")
        y_goal = rospy.get_param("y_goal")
        distance_tolerance = rospy.get_param("distance_tolerance")
        # Note: print() need declare output = "screen" in launch file
        print("Get param from launch file:\n x_goal: %s, y_goal: %s, Distance tolerance: %s" % (
            x_goal, y_goal, distance_tolerance ))

        bot.move_to_goal(x_goal, y_goal, distance_tolerance)

        
    except rospy.ROSInterruptException:
        pass
