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
#define TORQUE_SUB "torque"

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
float ksi = 1;
float Trsv = 4;
float wnv = 3.8/(ksi*Trsv);

float Trsw = 4;
float wnw = 3.8/(ksi*Trsw);

float K11 = pow(wnv,2);
float K12 = 2 * ksi *wnv;
float K13 = 0;
float K14 = 0;
float K21 = 0;
float K22 = 0;
float K23 = pow(wnw,2);
float K24 = 2 * ksi * wnw;

float Kr11 = pow(wnv,2) * 10;
float Kr12 = 0;
float Kr21 = 0;
float Kr22 = pow(wnw,2);

float mc = 100.0;
float mw = 7.5;
float m = mc + 2* mw;
float R = 0.165;
float L = 0.285;
float d = 0.25;
float Ic = 0.5 * mc * pow(d, 2);
float Im = mw * pow(R,2);
float Iw = mw * pow(L,2);
float I = (Ic + mc * pow(d,2) + 2 * mw * pow(L,2) + 2 * Im);

float K1 = -32.3;
float K2 = -200.8;
float B1 = 195.2;
float K3 = -32.3;
float K4 = -200.8;
float B2 = 195.2;
//float K1 = -34.3;
//float K2 = -246.8;
//float B1 = 180.2;
//float K3 = -30.3;
//float K4 = -163.1;
//float B2 = 213.5;


// TODO maybi change to a strukt. Global = bad!
float speed_reference = 0;
float steering_reference = 0;
float current_L_vel = 0;
float current_R_vel = 0;

float ar;
float al;
float v;
float w;

float tauR = 0;
float tauL = 0;

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
void refCallback(const geometry_msgs::Twist::ConstPtr& msg);
void vwCallback(const geometry_msgs::Twist::ConstPtr& msg);
void torqueCallback(const geometry_msgs::Twist::ConstPtr& msg);
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
void loopCallback(const ros::TimerEvent&);

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

void refCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	speed_reference = msg->linear.x;
	steering_reference = msg->angular.z;
	//ROS_INFO("refcallback %f %f", speed_reference, steering_reference);
}


// receives new data from joy
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

  	//test if button have been pressed
	toggleButton(msg->buttons[0], &handbreak);
 	toggleButton(msg->buttons[7], &startUp);
	emergency_override = (msg->buttons[1] == 1); //B Button

	joy_timer = getMilliCount();
	//ROS_INFO("joy callback");
}

void stopCallback(const std_msgs::Bool::ConstPtr& lidar_stop)
{
	coll_stop = lidar_stop->data;
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

void vwCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	v = msg->linear.x;
	w = msg->angular.z;
}

void torqueCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	tauL = msg->linear.x;
	tauR = msg->linear.y;
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
	else if (pwr_msg.linear.x < -100)
		pwr_msg.linear.x = -100;
	if (pwr_msg.linear.y > 100)
		pwr_msg.linear.y = 100;
	else if (pwr_msg.linear.y < -100)
		pwr_msg.linear.y = -100;
	motor_power_pub.publish(pwr_msg);
	//ROS_INFO("pub %f %f", pwr_msg.linear.x, pwr_msg.linear.y);
	return;
}

void arx(){
	//pwr_msg.linear.x = 9.61 * al + 11.45 * current_L_vel + 0.85 * last_msg.linear.x;
	pwr_msg.linear.x = 9.61 * al + 11.45 * current_L_vel + 0.85 * last_msg.linear.x;
	pwr_msg.linear.y = 10.95 * ar + 13.52 * current_R_vel + 0.83 * last_msg.linear.y; 
	//pwr_msg.linear.y = 10.95 * ar + 10.00 * current_R_vel + 0.83 * last_msg.linear.y; 
	//ROS_INFO("arx %f %f", speed_reference, steering_reference);
}


void feedBackLinerisation(){
/*
	pwr_msg.linear.y = - ((m * pow(R,2) + 2 * Iw) * (K11 * v - Kr11 * speed_reference + K13 * w - Kr12 
		* steering_reference + (R * (K1 * tauR + K2 * v + K2 * L * w))/(m * pow(R,2) + 2 * Iw) 
		+ (R * (K3 * tauL + K4 * v - K4 * L * w))/(m * pow(R,2) + 2 * Iw) + (K14 * R 
		* (L * tauR - L * tauL + R * d * mc * v * w))/(2 * Iw * pow(L,2) + I * pow(R,2)) 
		+ (K12 * R * (- R * d * mc * pow(w,2) + tauL + tauR))/(m * pow(R,2) + 2 * Iw) 
		- (2 * pow(R,3) * d * mc * w * (L * tauR - L * tauL + R * d * mc * v * w))/((2 * Iw 
		* pow(L,2) + I * pow(R,2)) * (m * pow(R,2) + 2 * Iw))))/(2 * B1 * R) - ((2 * Iw 
		* pow(L,2) + I * pow(R,2)) * (K21 * v - Kr21 * speed_reference + K23 * w - Kr22 
		* steering_reference + (L * R * (K1 * tauR + K2 * v + K2 * L * w))/(2 * Iw * pow(L,2) 
		+ I * pow(R,2)) - (L * R * (K3 * tauL + K4 * v - K4 * L * w))/(2 * Iw * pow(L,2) + I 
		* pow(R,2)) + (K24 * R * (L * tauR - L * tauL + R * d * mc * v * w))/(2 * Iw * pow(L,2) 
		+ I * pow(R,2)) + (K22 * R * (- R * d * mc * pow(w,2) + tauL + tauR))/(m * pow(R,2) + 2 
		* Iw) + (pow(R,3) * d * mc * v * (L * tauR - L * tauL + R * d * mc * v * w))/pow((2 * Iw 
		* pow(L,2) + I * pow(R,2)),2) + (pow(R,3) * d * mc * w * (- R * d * mc * pow(w,2) 
		+ tauL + tauR))/((2 * Iw * pow(L,2) + I * pow(R,2)) * (m * pow(R,2) + 2 * Iw))))
		/ (2 * B1 * L * R);

	pwr_msg.linear.x = ((2 * Iw * pow(L,2) + I * pow(R,2)) * (K21 * v - Kr21 * speed_reference + K23 * w 
		- Kr22 * steering_reference + (L * R * (K1 * tauR + K2 * v + K2 * L * w))/(2 * Iw
		* pow(L,2) + I * pow(R,2)) - (L * R * (K3 * tauL + K4 * v - K4 * L * w))/(2 * Iw
		* pow(L,2) + I * pow(R,2)) + (K24 * R * (L * tauR - L * tauL + R * d * mc * v * w))
		/ (2 * Iw * pow(L,2) + I * pow(R,2)) + (K22 * R * (- R * d * mc * pow(w,2) + tauL 
		+ tauR))/(m * pow(R,2) + 2 * Iw) + (pow(R,3) * d * mc * v * (L * tauR - L * tauL 
		+ R * d * mc * v * w))/pow((2 * Iw * pow(L,2) + I * pow(R,2)),2) + (pow(R,3) * d 
		* mc * w * (- R *d * mc * pow(w,2) + tauL + tauR))/((2 * Iw * pow(L,2) + I * pow(R,2))
		* (m * pow(R,2) + 2 *Iw))))/(2 * B2 * L * R) - ((m * pow(R,2) + 2 * Iw) * (K11 * v 
		- Kr11 * speed_reference + K13 * w - Kr12 * steering_reference + (R * (K1 * tauR 
		+ K2 * v + K2 * L * w))/(m * pow(R,2) + 2 * Iw) + (R * (K3 * tauL + K4 * v - K4 * L 
		* w))/(m * pow(R,2) + 2 * Iw) + (K14 * R * (L * tauR - L * tauL + R * d * mc * v * w))
		/ (2 * Iw * pow(L,2) + I * pow(R,2)) + (K12 * R * (- R * d * mc * pow(w,2) + tauL 
		+ tauR))/(m * pow(R,2) + 2 * Iw) - (2 * pow(R,3) * d * mc * w * (L * tauR - L * tauL 
		+ R * d * mc * v * w))/((2 * Iw * pow(L,2) + I * pow(R,2)) * (m * pow(R,2) + 2 * Iw))))
		/ (2 * B2 *R);

*/

	pwr_msg.linear.y = 0.116*tauR + 0.838*v + 0.0424*speed_reference + 0.237*w + 0.0136*steering_reference - 0.0491*v*w - 9.8 * pow(10,-4) * w*(- 4.12 * pow(w,2) + tauL + tauR) + 0.014 * pow (w,2) - 0.0107*v*(0.285*tauR - 0.285*tauL + 4.12*v*w) + 0.0061*w*(0.285*tauR - 0.285*tauL + 4.12*v*w);
	pwr_msg.linear.x = 0.133*tauL + 0.708*v + 0.0557*speed_reference - 0.2*w - 0.0179*steering_reference + 0.0644*v*w + 0.00129*w*(- 4.12 * pow(w,2) + tauL + tauR) + 0.0184 * pow(w,2) + 0.014*v*(0.285*tauR - 0.285*tauL + 4.12*v*w) + 0.008*w*(0.285*tauR - 0.285*tauL + 4.12*v*w);


	pwr_msg.linear.y = pwr_msg.linear.y * 100;
	pwr_msg.linear.x = pwr_msg.linear.x * 100;

}


// looping trigerd at desierd freques.
void loopCallback(const ros::TimerEvent&)
{
	setVelMsg();
	feedBackLinerisation();
	emergencyStop(); 
	pubEnginePower();
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, NODE_NAME);

	ros::NodeHandle n;
	ros::NodeHandle nh("~");
  

	nh.param<int>("power_buffer_size",POWER_BUFFER_SIZE,200);
	nh.param<int>("encoder_buffer_size",ENCODER_BUFFER_SIZE,5);
	nh.param<int>("loop_freq",LOOP_FREQ,100);
	nh.param<int>("time_out",TIME_OUT,500);
	
	//ROS_INFO("time_out %d", TIME_OUT);
	
	//set up communication channels
	motor_power_pub = n.advertise<geometry_msgs::Twist>(ADVERTISE_POWER, POWER_BUFFER_SIZE);
	ros::Subscriber joysub = n.subscribe<sensor_msgs::Joy>(JOY_SUB_NODE, POWER_BUFFER_SIZE, joyCallback); 
	ros::Subscriber wheel_vel_sub = n.subscribe<std_msgs::Float32MultiArray>(SUBSCRIBE_ENCODER, ENCODER_BUFFER_SIZE, encoderCallback);
	ros::Subscriber stop_sub = n.subscribe<std_msgs::Bool>(STOP_SUB_NODE, 1, stopCallback);
	ros::Subscriber vw_encder = n.subscribe<geometry_msgs::Twist>(VW_SUB, ENCODER_BUFFER_SIZE, vwCallback);
	ros::Subscriber refens = n.subscribe<geometry_msgs::Twist>("referens", POWER_BUFFER_SIZE, refCallback);
	ros::Subscriber torque = n.subscribe<geometry_msgs::Twist>(TORQUE_SUB, ENCODER_BUFFER_SIZE, torqueCallback);

	ros::Timer loop = n.createTimer(ros::Duration(0.01), loopCallback);
	// for diagnostik print
  /* Data we have: vel_msg.linear.x - wanted speeds on wheels, ie r
  *  		   current_L_vel, ie y
  */

 	ros::spin();

	return 0;
}

		// %EndTag(FULLTEXT)%
