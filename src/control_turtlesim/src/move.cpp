#include "ros/ros.h"
#include "geometry_msgs/Twist.h"    // Pub: send move message
#include "turtlesim/Pose.h" // Sub: receive position
#include "iostream"


// Confirm command
// rostopic list
// rostopic info /turtle1/cmd_vel
// rosmsg show geometry_msgs/Twist


turtlesim::Pose turtlesim_pose;

// Move Strain
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


// Rotate
void rotate(ros::Publisher velocity_publisher, double angular_speed_degree, 
            double relative_angle_degree, bool clockwise)
{   
    // Create message obj
    geometry_msgs::Twist velocity_message;

    double angular_speed = angular_speed_degree / (180.0/3.141592653589793238463);

    if(clockwise){
        velocity_message.angular.z = -abs(angular_speed);
    } else {
        velocity_message.angular.z = abs(angular_speed);
    }

    // Time to caculate
    double t0 = ros::Time::now().toSec();
    double current_angle_degree;
    ros::Rate loop_rate(100);

    // Pub request move message
    do{
        velocity_publisher.publish(velocity_message);
        double t1 = ros::Time::now().toSec();
        current_angle_degree = angular_speed_degree * (t1 - t0); // Caculate current angle from speed and time
        ros::spinOnce();
        loop_rate.sleep();
    } while (current_angle_degree < relative_angle_degree );

    // Stop
    velocity_message.angular.z = 0;
    velocity_publisher.publish(velocity_message);

}


// Go to goal

double getDistance(double x1, double y1, double x2, double y2){
    return sqrt( pow((x1-x2), 2) + pow( y1-y2, 2) );
}

void go_to_goal(ros::Publisher velocity_publisher)
{   
    // Get input from user
    turtlesim::Pose goal_pose;
    double distance_tolerance;
    std::cout << "Goal x: ";
    std::cin >> goal_pose.x;
    std::cout << "Goal y: ";
    std::cin >> goal_pose.y;
    std::cout << "Distance torelance: ";
    std::cin >> distance_tolerance;

    // Create message obj
    geometry_msgs::Twist vel_msg;

    ros::Rate loop_rate(100);
    double E = 0.0;

    do {
        double Kp = 1.0;
        double Ki = 0.02;
        double e = getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
        double E = E+e;

        vel_msg.linear.x = (Kp*e);
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;

        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = 4*( atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x-turtlesim_pose.x) - turtlesim_pose.theta);

        velocity_publisher.publish(vel_msg);

        ros::spinOnce();
        loop_rate.sleep();

    } while( getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)>distance_tolerance );

    std::cout << "End move goal";
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);

}


// Call back function
void poseCallback(const turtlesim::Pose::ConstPtr &pose_message){

    float x {pose_message->x};
    float y {pose_message->y};
    float theta {pose_message->theta};

    ROS_INFO("x: %f", x ); // %f :float
    ROS_INFO("y: %f", y );
    ROS_INFO("theta: %f", theta );

    // Save to global scope turtlesim_pose
    turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;

}


int main(int argc, char **argv)
{
    // Init new node with name "move_cpp"
    ros::init(argc, argv, "move_cpp");
    ros::NodeHandle n;
    double speed, angular_speed;
    double distance, angle;
    bool isForward, clockwise;

    // Declare Pub v
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

    // Declare Sub to show location
    ros::Subscriber pose_subscribe = n.subscribe("/turtle1/pose", 10, poseCallback);

    // Test code
    // Move Strain
    // move(velocity_publisher, 1, 2, true);

    // Rotate
    // rotate(velocity_publisher, 10, 90, true);

    // Go to goal
    go_to_goal(velocity_publisher);

    // Wait
    ros::Rate loop_rate(0.5);
    ros::spin();

    return 0;
}