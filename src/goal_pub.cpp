#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv){
ros::init(argc, argv, "Goal_Pub");
ros::NodeHandle nh;

ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/jackal/target/position", 1000);

ros::Rate rate(10);
float x = 4.0;
float y = 4.0;
std::cout << "Sending goal position X = " << x << "\t Y = " << y << std::endl;

while(ros::ok()){
geometry_msgs::Twist msg;
msg.linear.x = x;
msg.linear.y = y;
msg.linear.z = 0.0;

pub.publish(msg);
rate.sleep();
}

ros::spin();
}
