// Useful tutorial for writing sub and pub with class https://github.com/wsnewman/ros_class/blob/master/example_ros_class/src/example_ros_class.h

#include <ros/ros.h>
#include <vector>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
// #include "tf/LinearMath/Transform.h"
#include <tf/transform_listener.h>

class Goal_Pub {

private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;

public:
  // Data members
  float goal_x;
  float goal_y;

  // Member functions
  // Constructor
  Goal_Pub(){
    init_sub();
    init_pub();
    goal_x = 5.0;
    goal_y = 5.0;
    ROS_INFO("Inside Contructor");
  }

  void init_sub() {
    ROS_INFO("Initializing Subscribers");
    sub = nh.subscribe("/gazebo/model_states", 100, &Goal_Pub::stateCb, this);
  }

  void init_pub() {
    ROS_INFO("Initializing Publishers");
    pub = nh.advertise<geometry_msgs::Twist>("/jackal/target/position", 100, true);

  }

  void stateCb(const gazebo_msgs::ModelStates& msg) {
	geometry_msgs::Twist rel_goal;
  for(int i = 0; i < msg.pose.size(); i++) {
    if (msg.name[i].substr(0, 6) == std::string("jackal")){
      // std::cout<< "Jackal Position = " << msg.pose[i].position.x << "   " << msg.pose[i].position.y << std::endl;
      rel_goal.linear.x = goal_x - msg.pose[i].position.x;
      rel_goal.linear.y = goal_y - msg.pose[i].position.y;

      tf::Pose pose;
      tf::poseMsgToTF(msg.pose[i], pose);
      double yaw = tf::getYaw(pose.getRotation());
      // std::cout<<" Yaw = " << yaw * 57.296 << std::endl;
      // yaw = yaw * 57.296;
      rel_goal.linear.z = yaw;
      pub.publish(rel_goal);
      }
    }
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv,"Rel_Goal_Pub");
  Goal_Pub obj;
  ros::spin();
  return 0;
}
