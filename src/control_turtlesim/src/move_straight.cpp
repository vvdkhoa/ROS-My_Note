#include "ros/ros.h"
#include "geometry_msgs/Twist.h"    // Pub: send move message
#include "turtlesim/Pose.h" // Sub: receive position
#include "iostream"


// Confirm command
// rostopic list
// rostopic info /turtle1/cmd_vel
// rosmsg show geometry_msgs/Twist


void move(ros::Publisher velocity_publisher, double speed, double distance, bool isForward)
{
    geometry_msgs::Twist vel_msg;

    if(isForward) {
        vel_msg.linear.x = abs(speed);
    } else {
        vel_msg.linear.x = -abs(speed);
    }
    
    double t0 = ros::Time::now().toSec();
    double current_distance = 0.0;
    ros::Rate loop_rate(100);
    do {
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_distance = speed * (t1-t0); // s = v.(t1-t0)
        ros::spinOnce();
        loop_rate.sleep();
    } while (current_distance < distance);

    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);

}

void poseCallback(const turtlesim::Pose::ConstPtr &pose_message){

    float x {pose_message->x};
    float y {pose_message->y};
    float theta {pose_message->theta};

    ROS_INFO("x: %f", x ); // %f :float
    ROS_INFO("y: %f", y );
    ROS_INFO("theta: %f", theta );

}

int main(int argc, char **argv)
{
    // Init new node with name "move_straight_cpp"
    ros::init(argc, argv, "move_straight_cpp");
    ros::NodeHandle n;
    double speed, angular_speed;
    double distance, angle;
    bool isForward, clockwise;

    // Declare Pub v
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

    // Declare Sub to show location
    ros::Subscriber pose_subscribe = n.subscribe("/turtle1/pose", 10, poseCallback);

    // Test code
    move(velocity_publisher, 1, 4, false);

    // Wait
    ros::Rate loop_rate(0.5);
    ros::spin();

    return 0;
}