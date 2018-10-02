
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include <time.h>
#include <math.h>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Bool.h"

//constant setup variables change those values here
#define NODE_NAME "engine_mgmt"
#define JOY_SUB_NODE "joy"
#define STOP_SUB_NODE "lidar_stop"
#define ADVERTISE_POWER "motor_power" //publishing channel
#define POWER_BUFFER_SIZE 200
#define SUBSCRIBE_ENCODER "wheel_velocity"
#define ENCODER_BUFFER_SIZE 5
#define LOOP_FREQ 20

//#define PUSH_SPEED 250 // minimal ms between toggle button register
#define TIME_OUT 500 // if no joy msg arrives in this time (ms) will it stop TODO test

// joy msg->axes array layout; for a x-box 360 controller
// [left stick RL(0) , left stick up/down(1), LT(2) ,right stick RL(3) , right stick up/down(4) , RT(5) , pad RL(6), pad up/down(7) ]
// joy msg->button array layout
// [ A, B, X, Y, LB, RB, back, start, XBOX, L-stick, R-stick]


// message declarations
geometry_msgs::Twist vel_msg;
geometry_msgs::Twist pwr_msg;
ros::Publisher motor_power_pub;

std_msgs::Float32MultiArray wheel_velocities;

// TODO maybe change to a struct. Global = bad!
float speed_reference = 0;
float steering_reference = 0;
float current_L_vel = 0;
float current_R_vel = 0;

double joy_timer;

bool coll_stop;
bool emergency_override;

struct toggleButton {
	bool on;
	bool previews;
};
struct toggleButton handbreak = {.on = false, .previews = false}; // A 
struct toggleButton startUp = {.on = true, .previews = false}; // start
int pressed_button;

void pubEnginePower();
void encoderCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
float inputSens(float ref);
void toggleButton(int val, struct toggleButton *b);
void emergencyStop();
// temporary functions might change or be replaced when real controller implements
void sterToSpeedBalancer();
void setVelMsg();
void PID(float *Le, float *Re, float *Lle, float *Rle,
		 float *Lea, float *Rea, float P, float D,
		 float I, float *uL, float *uR, float updatefreq);

// receives new data from joy
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  // read input form joy
	speed_reference = 50 * (-msg->axes[5] + msg->axes[2]);
	steering_reference = inputSens(msg->axes[0]); 

  	//test if button have been pressed
	toggleButton(msg->buttons[0], &handbreak);
 	toggleButton(msg->buttons[7], &startUp);
	pressed_button = msg->buttons[1];
	
	overrideButton();

	joy_timer = clock();
}

void stopCallback(const std_msgs::Bool::ConstPtr& lidar_stop)
{
	coll_stop = lidar_stop->data;
}

void overrideButton(){
	if(pressed_button == 1) emergency_override = true;
	else emergency_override = false;
}


// adjust input sensitivity
float inputSens(float ref){
	return pow(ref, 3) * 100;
}

// loadig to handle buttons that should toggle
void toggleButton(int val, struct toggleButton *b){
		if (val == 1 && !b->previews)
			b->on = !b->on;
		b->previews = (val == 1);
}

// ensure that the speed don't exist full power
void sterToSpeedBalancer(){
	while (abs(speed_reference) + abs(steering_reference) > 100){
		if (speed_reference > 0) speed_reference--;
		else speed_reference++;
		if (steering_reference > 0) steering_reference--;
		else steering_reference++;
	}
	return;
}

void encoderCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	/* idea: reference for both wheels come from joy, processed and globally declared variables
	*			The encoders provide feedback in this function, where we then call a PID function for 
	*			each wheel independently,
	*			Then we publish the new motor powers.
	*/

	current_L_vel = array->data[0];
	current_R_vel = array->data[1];
	//ROS_INFO("L_speed %f\n",current_L_vel);

}

// set the new values to the message
void setVelMsg(){
	
	if (((joy_timer - clock()) < TIME_OUT) && !startUp.on && (!coll_stop || emergency_override))
	{
		// changes if in reverse to not have inverted steering in reverse
		if (speed_reference < 0) steering_reference = -steering_reference;
		vel_msg.linear.x = speed_reference - steering_reference;
		vel_msg.linear.y = speed_reference + steering_reference;

	}
	else
	{
		vel_msg.linear.x = 0;
		vel_msg.linear.y = 0;
	}
	return;
}

// Emergency stop force to stop
void emergencyStop(){
	if (handbreak.on){
		vel_msg.linear.x = 0;
		vel_msg.linear.y = 0;
	}
}

// publish when new steering directions are set
void pubEnginePower()
{
	motor_power_pub.publish(pwr_msg);
	return;
}

// PID
void PID(float *Le, float *Re, float *Lle, float *Rle,
		 float *Lea, float *Rea, float P, float D,
		 float I, float *uL, float *uR, float updatefreq){

	sterToSpeedBalancer();
  	setVelMsg();
	
	//PID

	*Lle = *Le;
	*Rle = *Re;
	*Le =  vel_msg.linear.x/100 - current_L_vel;
	*Re =  vel_msg.linear.y/100 - current_R_vel;
	
	//	if Lea+Le
	*Lea = *Le + *Lea;
	*Rea = *Re + *Rea;
	*uL = P * *Le + D * updatefreq * (*Le- *Lle) + I* *Lea;
	*uR = P * *Re + D * updatefreq * (*Re- *Rle) + I* *Rea; 

	if(*uL>100) *uL=100;
	else if(*uL<-100) *uL=-100;
	if(*uR>100) *uR=100;
	else if(*uR<-100) *uR=-100;

	pwr_msg.linear.x = *uL + vel_msg.linear.x * 0.5;
	pwr_msg.linear.y = *uR + vel_msg.linear.y * 0.5;

	ROS_INFO("rL: %f, Le: %f, Lle: %f, Lea: %f",vel_msg.linear.x/100, *Le, *Lle, *Lea);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, NODE_NAME);

  ros::NodeHandle n;
  ros::Rate loop_rate(LOOP_FREQ);
  
  //set up communication channels
  // TODO add the other channels
  motor_power_pub = n.advertise<geometry_msgs::Twist>(ADVERTISE_POWER, POWER_BUFFER_SIZE);
  ros::Subscriber joysub = n.subscribe<sensor_msgs::Joy>(JOY_SUB_NODE, POWER_BUFFER_SIZE, joyCallback); 
  ros::Subscriber wheel_vel_sub = n.subscribe<std_msgs::Float32MultiArray>(SUBSCRIBE_ENCODER, ENCODER_BUFFER_SIZE, encoderCallback);
  ros::Subscriber stop_sub = n.subscribe<std_msgs::Bool>(STOP_SUB_NODE, 1, stopCallback);
  //ros::spin();

  /* Data we have: vel_msg.linear.x - wanted speeds on wheels, ie r
  *  		   current_L_vel, ie y
  */


  float Le=0, Re=0, Lle=0, Rle=0, Lea=0, Rea=0, P=70, D=0, I=15, uL=0, uR=0;
  float updatefreq = LOOP_FREQ;

  while(ros::ok()){

	PID(&Le, &Re, &Lle, &Rle, &Lea, &Rea, P, D, I, &uL, &uR, updatefreq);
	

	emergencyStop();
	pubEnginePower();
  	ros::spinOnce();
  	loop_rate.sleep();

  }

  return 0;
}

// %EndTag(FULLTEXT)%
