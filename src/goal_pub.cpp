#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// void filterVelocityCallback(const geometry_msgs::Twist& msg){
// if (msg.angular.z > 0){
// ROS_INFO_STREAM("Subscriber velocities:"<<" linear="<<msg.linear.x<<" angular="<<msg.angular.z);
// }
// }

int main(int argc, char **argv){
ros::init(argc, argv, "Goal_Pub");
ros::NodeHandle nh;

// ros::Subscriber sub = nh.subscribe("turtle1/cmd_vel",1000,&filterVelocityCallback);
ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/jackal/target/position", 1000);

ros::Rate rate(10);

while(ros::ok()){
geometry_msgs::Twist msg;
msg.linear.x = 10.0;
msg.linear.y = 10.0;
msg.linear.z = 0.0;

pub.publish(msg);
// ROS_INFO_STREAM("Filtered velocities:"<<" linear="<<msg.linear.x<<" angular="<<msg.angular.z);
rate.sleep();
}

ros::spin();
}
