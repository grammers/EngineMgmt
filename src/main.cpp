
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"


geometry_msgs::Twist vel_msg;
ros::Publisher motor_power_pub;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  vel_msg.linear.x = 100*(msg->axes[1]);
  vel_msg.linear.z = 100*(msg->axes[4]);

  motor_power_pub.publish(vel_msg);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "EngineMgmt");

  ros::NodeHandle n;

vel_msg.linear.x = 0;
vel_msg.linear.y = 0;
vel_msg.linear.z = 0;
vel_msg.angular.x = 0;
vel_msg.angular.y = 0;
vel_msg.angular.z = 0;

  motor_power_pub = n.advertise<geometry_msgs::Twist>("motor_power", 200);
  ros::Subscriber joysub = n.subscribe<sensor_msgs::Joy>("joy", 200, joyCallback);

  ros::spin();

  return 0;
}
// %EndTag(FULLTEXT)%
