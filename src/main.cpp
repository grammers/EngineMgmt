
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include <sys/timeb.h>
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
#define SUBSCRIBE_ENCODER "wheel_velocity"
#define VW_SUB "vw_estimate"

// joy msg->axes array layout; for a x-box 360 controller
// [left stick RL(0) , left stick up/down(1), LT(2) ,right stick RL(3) , right stick up/down(4) , RT(5) , pad RL(6), pad up/down(7) ]
// joy msg->button array layout
// [ A, B, X, Y, LB, RB, back, start, XBOX, L-stick, R-stick]


// message declarations
geometry_msgs::Twist pwr_msg;
geometry_msgs::Twist last_msg;
ros::Publisher motor_power_pub;

std_msgs::Float32MultiArray wheel_velocities;

// setings
int POWER_BUFFER_SIZE;
int ENCODER_BUFFER_SIZE;
int LOOP_FREQ;
int TIME_OUT;
// feedBackLineraiion
float k11;
float k22;
float kr11;
float kr22;


// TODO maybi change to a strukt. Global = bad!
float speed_reference = 0;
float steering_reference = 0;
float current_L_vel = 0;
float current_R_vel = 0;

float ar;
float al;
float v;
float w;

int joy_timer;

bool coll_stop;
bool emergency_override;

struct toggleButton {
	bool on;
	bool previews;
};
struct toggleButton handbreak = {.on = false, .previews = false}; // A 
struct toggleButton startUp = {.on = true, .previews = false}; // start


void pubEnginePower();
void encoderCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
void vwCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
float inputSens(float ref);
void toggleButton(int val, struct toggleButton *b);
void emergencyStop();
// temporary functions might change or be replaced when real controller implements
void setVelMsg();
//time functions
int getMilliCount();
int getMilliSpan(int nTimeStart);
// controll loop
void feedBackLinerisation();
void arx();

// Calculate time in ms
int getMilliCount(){
	timeb tb;
	ftime(&tb);
	int nCount = tb.millitm + (tb.time & 0xfffff) * 1000;
	return nCount;
}

// difremt in ms from nTimeStart wos taken
int getMilliSpan(int nTimeStart){
	int nSpan = getMilliCount() - nTimeStart;
	if(nSpan < 0)
		nSpan += 0x100000 * 1000;
	return nSpan;
}

// receives new data from joy
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  // read input form joy
	speed_reference = (-msg->axes[5] + msg->axes[2]) / 2;
	steering_reference = inputSens(msg->axes[0]); 

  	//test if button have been pressed
	toggleButton(msg->buttons[0], &handbreak);
 	toggleButton(msg->buttons[7], &startUp);
	emergency_override = (msg->buttons[1] == 1); //B Button

	joy_timer = getMilliCount();
	if (speed_reference < 0) steering_reference = -steering_reference;
	//ROS_INFO("joy callback");
}

void stopCallback(const std_msgs::Bool::ConstPtr& lidar_stop)
{
	coll_stop = lidar_stop->data;
}

// adjust input sensitivity
float inputSens(float ref){
	if (ref >= 0)
		return pow(ref, 2);
	else
		return -pow(ref,2);
}

// loadig to handle buttons that should toggle
void toggleButton(int val, struct toggleButton *b){
		if (val == 1 && !b->previews)
			b->on = !b->on;
		b->previews = (val == 1);
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

void vwCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	v = array->data[0];
	w = array->data[1];
}

// set the new values to the message
void setVelMsg(){
	
	last_msg = pwr_msg;
	if ((getMilliSpan(joy_timer) > TIME_OUT) || startUp.on || (coll_stop && !emergency_override))
	{
		speed_reference = 0;
		steering_reference = 0;
	}
	//bool temp = getMilliSpan(joy_timer) < TIME_OUT;
	//ROS_INFO("span: %d", getMilliSpan(joy_timer));
	//ROS_INFO("msg_ref_set pre control X: %f, Y: %f, strtUp: %d, coll: %d, overtide %d, clock_out: %d", pwr_msg.linear.x, pwr_msg.linear.y, startUp.on, coll_stop, emergency_override, temp);
	return;
}

// Emergency stop force to stop
void emergencyStop(){
	if	(handbreak.on){
		pwr_msg.linear.x = 0;
		pwr_msg.linear.y = 0;
	}
}

// publish when new steering directions are set
void pubEnginePower()
{
	if (pwr_msg.linear.x > 100)
		pwr_msg.linear.x = 100;
	if (pwr_msg.linear.y > 100)
		pwr_msg.linear.y = 100;
	motor_power_pub.publish(pwr_msg);
	return;
}

void arx(){
	//pwr_msg.linear.x = 9.61 * al + 11.45 * current_L_vel + 0.85 * last_msg.linear.x;
	pwr_msg.linear.x = 9.61 * al + 11.45 * current_L_vel + 0.85 * last_msg.linear.x;
	pwr_msg.linear.y = 10.95 * ar + 13.52 * current_R_vel + 0.83 * last_msg.linear.y; 
	//pwr_msg.linear.y = 10.95 * ar + 10.00 * current_R_vel + 0.83 * last_msg.linear.y; 
}


void feedBackLinerisation(){
	ar = 1.2030 * kr11 * speed_reference - 1.2030 * k11 * v - 0.3817 * k22 * w + 0.3817 * kr22 * steering_reference + 0.4386 * v * w - 0.1250 * pow(w, 2); 
	al = 1.2030 * kr11 * speed_reference - 1.2030 * k11 * v + 0.3817 * k22 * w - 0.3817 * kr22 * steering_reference - 0.4386 * v * w - 0.1250 * pow(w, 2); 
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, NODE_NAME);

	ros::NodeHandle n;
	ros::NodeHandle nh("~");
  

	nh.param<int>("power_buffer_size",POWER_BUFFER_SIZE,200);
	nh.param<int>("encoder_buffer_size",ENCODER_BUFFER_SIZE,5);
	nh.param<int>("loop_freq",LOOP_FREQ,20);
	nh.param<int>("time_out",TIME_OUT,500);
	nh.param<float>("k11",k11,1.5);
	nh.param<float>("k22",k22,2.5);
	nh.param<float>("kr11",kr11,1.5);
	nh.param<float>("kr22",kr22,5);
	
	//ROS_INFO("time_out %d", TIME_OUT);
	
	ros::Rate loop_rate(LOOP_FREQ);
	//set up communication channels
	motor_power_pub = n.advertise<geometry_msgs::Twist>(ADVERTISE_POWER, POWER_BUFFER_SIZE);
	ros::Subscriber joysub = n.subscribe<sensor_msgs::Joy>(JOY_SUB_NODE, POWER_BUFFER_SIZE, joyCallback); 
	ros::Subscriber wheel_vel_sub = n.subscribe<std_msgs::Float32MultiArray>(SUBSCRIBE_ENCODER, ENCODER_BUFFER_SIZE, encoderCallback);
	ros::Subscriber stop_sub = n.subscribe<std_msgs::Bool>(STOP_SUB_NODE, 1, stopCallback);
	ros::Subscriber vw_encder = n.subscribe<std_msgs::Float32MultiArray>(VW_SUB, ENCODER_BUFFER_SIZE, vwCallback);

	// for diagnostik print
  /* Data we have: vel_msg.linear.x - wanted speeds on wheels, ie r
  *  		   current_L_vel, ie y
  */

	while(ros::ok()){
  		ros::spinOnce();
		setVelMsg();
		feedBackLinerisation();
		arx();
		emergencyStop(); 
		pubEnginePower();
  		loop_rate.sleep();

	}

	return 0;
}

		// %EndTag(FULLTEXT)%
