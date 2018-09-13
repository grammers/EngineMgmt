// %Tag(FULLTEXT)%
#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class EngienMgmt
{
	public:
		EngienMgmt();

	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	
		
	ros::NodeHandle n_;
	int linear_. angular_;
	double l_scale_, a_scale_;
	ros::Publisher vel_pub_;
	ros::Subscriber joy;

};

EngienMgmt::EngienMgmt():
	linar_(1),
	angular_(2)
{
	n_.param("axis_linear", linear_, linear_);
	n_.param("axis_angular", angular_, angular_);
	n_.param("scale_angular", a_scale_, a_scale_);
	n_.param("scale_linear", l_scale_, l_scale_);

	vel_pub_ = n_.advertise<geometry_msgs::Twist>("motor_power", 1);

	joy_sub_ = n_.Subscriber<sensor_msgs::Joy>("joy", 10, &EngienMgmt::engCallback, this);
}
			
//#include "string"
//#include <std::string::substr()>
//#include <std::string::find()>
/*
struct JOJ_INPUT {
	double speed;
	double diraction;
} driving_instruktions;

JOJ_INPUT g_latest_input;

// %Tag(CALLBACK)%
void padPosLog (const std_msgs::String::ConstPtr & msg)
{
	std_msgs::String s;
        s.data	= msg->data;
	std::string delimiter = "{";
	std::string input [4];
	input[0] = s.data.std::string::substr(0, s.data.std::string::find(delimiter));
	std::cout<<input[0]<<std::endl;
	ROS_INFO("I herd: [%s]", msg->data.c_str());
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  ros::init(argc, argv, "engien_mgmt");
  ros::NodeHandle n;

  // %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("joy", 1, padPosLog);
// %EndTag(SUBSCRIBER)%
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%
*/
