
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include <time.h>

//constant setup variabels change thise values here
#define NODE_NAME "engine_mgmt"
#define JOY_SUB_NODE "joy"
#define ADVERTISE_POWER "motor_power"
#define BUFER_SIZE 200
//#define PUSH_SPEED 250 // minimal ms betvin toggel buton register
#define TIME_OUT 500 // if no joy msg arives in thise time (ms) will it stop TODO test

// joy msg->axes array lay out; for a x-box 360 controller
// [ , left stik upp/down, , , right stik upp/down , , ]
// joy msg->button array lay out
// [ A, 


// message deglaraions
geometry_msgs::Twist vel_msg;
ros::Publisher motor_power_pub;

float x_referens = 0;
float y_referens = 0;

bool stop_button = false;

int s_singel_press = 0;
double joy_timer;

void pubEnginePower();

// reseves new data from joy
// TODO stor data and macke calebrations for mor fansy controling befor publiching
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  // read input form joj
  	x_referens = 100*(msg->axes[1]);
  	y_referens = 100*(msg->axes[4]);

  //test if button hav ben presed
	if (msg->buttons[0] == 1 && s_singel_press != msg->buttons[0])
 	{
		stop_button = !stop_button;
 	}

	s_singel_press = msg->buttons[0];
 	joy_timer = clock();
 	pubEnginePower();	//TODO move to location  where control lodgik wont it to be called
}

// publiche when new stering diraktions is set
void pubEnginePower()
{
	if (((joy_timer - clock()) < TIME_OUT) && !stop_button)
	{
		vel_msg.linear.x = x_referens;
		vel_msg.linear.z = y_referens; //TODO change to y axes
	}
	else
	{
		vel_msg.linear.x = 0;
		vel_msg.linear.z = 0;
	}
	motor_power_pub.publish(vel_msg);
	return;
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
