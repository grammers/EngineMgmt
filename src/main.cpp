
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"


//constant setup variabels change thise values here
#define NODE_NAME "engien_mgmt"
#define JOY_SUB_NODE "joy"
#define ADVERTISE_POWER "motor_power"
#define BUFER_SIZE 200


geometry_msgs::Twist vel_msg;
ros::Publisher motor_power_pub;



// reseves new data from joy
// TODO stor data and macke calebrations for mor fansy controling befor publiching
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  vel_msg.linear.x = 100*(msg->axes[1]);
  vel_msg.linear.z = 100*(msg->axes[4]);

  motor_power_pub.publish(vel_msg);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, NODE_NAME);

  ros::NodeHandle n;
  
  //set upp comunication chanels
  // TODO add the other chanels
  motor_power_pub = n.advertise<geometry_msgs::Twist>(ADVERTISE_POWER, BUFER_SIZE);
  ros::Subscriber joysub = n.subscribe<sensor_msgs::Joy>(JOY_SUB_NODE, BUFER_SIZE, joyCallback); 
  ros::spin();

  return 0;
}
// %EndTag(FULLTEXT)%
