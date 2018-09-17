
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
// [left stik RL(0) , left stik upp/down(1), LT(2) ,right stik RL(3) , right stik upp/down(4) , RT(5) , pad RL(6), pad upp/down(7) ]
// joy msg->button array lay out
// [ A, B, X, Y, LB, RB, back, start, XBOX, L-stik, R-stik]


// message deglaraions
geometry_msgs::Twist vel_msg;
ros::Publisher motor_power_pub;

float speed_referens = 0;
float stering_referens = 0;

bool stop_button = false;

int s_singel_press = 0;
double joy_timer;

void pubEnginePower();
void sterToSpeedBallanser();
void setVelMsg();

// reseves new data from joy
// TODO stor data and macke calebrations for mor fansy controling befor publiching
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  // read input form joj
  	speed_referens = 50 * (-msg->axes[5] + msg->axes[2]);
	stering_referens = 100 * (msg->axes[0]);

  //test if button hav ben presed
	if (msg->buttons[0] == 1 && s_singel_press != msg->buttons[0])
 	{
		stop_button = !stop_button;
 	}

	s_singel_press = msg->buttons[0];
 	joy_timer = clock();
	
 	pubEnginePower();	//TODO move to location  where control lodgik wont it to be called
}

// enshor that the speed dont exsit full power
void sterToSpeedBallanser(){
	while (abs(speed_referens) + abs(stering_referens) > 100){
		if (speed_referens > 0) speed_referens--;
		else speed_referens++;
		if (stering_referens > 0) stering_referens--;
		else stering_referens++;
	}
	return;
}

// set a the new walues to the messge
void setVelMsg(){
	sterToSpeedBallanser();
	if (((joy_timer - clock()) < TIME_OUT) && !stop_button)
	{
		vel_msg.linear.x = speed_referens + stering_referens;
		vel_msg.linear.y = speed_referens - stering_referens;
	}
	else
	{
		vel_msg.linear.x = 0;
		vel_msg.linear.y = 0;
	}
	return;
}

// publiche when new stering diraktions is set
void pubEnginePower()
{
	setVelMsg();
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
