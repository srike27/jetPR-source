#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "sensor_msgs/JointState.h"

#include <sstream>

int servo_angle = 0;


void servoCallback(const std_msgs::Int16 msg)
{
  ::servo_angle = msg.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_updater");
  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
  ros::Subscriber sub = n.subscribe("servo", 1000, servoCallback);
  ros::Rate loop_rate(50);
  int count = 0;
  while (ros::ok())
  {
    sensor_msgs::JointState state = sensor_msgs::JointState();
    state.header.stamp = ros::Time::now();
    state.header.seq = count;
    state.name = {"left","right","camera_base_joint"};
    state.position = {0.0,0.0,0.0};
    float servo_angle_rad = -(::servo_angle*3.1415)/(180.0);
    state.position[2] = servo_angle_rad;
    joint_pub.publish(state);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
