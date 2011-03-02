#include <ros/ros.h>
#include <joy/Joy.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>

class CreateTeleop
{
  public:
    CreateTeleop();

		
  private:
    void joyCallback(const joy::Joy::ConstPtr& joy);
 		void sendTwist(const ros::TimerEvent& e);
 		
    ros::NodeHandle nh_;
    
    float linear, linearPrev;
    float angular, angularPrev;

    int linear_, angular_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    ros::Timer sendTimer;
};

CreateTeleop::CreateTeleop():
  linear_(1),
  angular_(2)
{
	linear      = 0.0f;
	linearPrev  = 0.0f;
	angular     = 0.0f;
	angularPrev = 0.0f;
	
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("create_node/cmd_vel", 1);

  joy_sub_ = nh_.subscribe<joy::Joy>("joy", 10, &CreateTeleop::joyCallback, this);
  
  sendTimer = nh_.createTimer(ros::Duration(0.1), &CreateTeleop::sendTwist, this);
  
}

void CreateTeleop::joyCallback(const joy::Joy::ConstPtr& joy)
{
	// Read the joystick positions & reverse the x axes so the coordinates are in standard orientation
	float y1 = 		    joy->axes[1];
	float x1 = 1.0f * joy->axes[0];
	float y2 = 		    joy->axes[3];
	float x2 = 1.0f * joy->axes[2];

	//printf("y1: %f x1: %f y2: %f x2: %f\n", y1, x1, y2, x2);
	
	linearPrev = linear;
	linear = y1;
	angularPrev = angular;
	angular = x1;

}

void CreateTeleop::sendTwist(const ros::TimerEvent& e)
{
	geometry_msgs::Twist twist;
	geometry_msgs::Vector3 twistLinear;
	geometry_msgs::Vector3 twistAngular;
	twistLinear.x = linear*0.75f + linearPrev*0.25f;
	twistLinear.y = 0;
	twistLinear.z = 0;
	
	twistAngular.x = 0;
	twistAngular.y = 0;
	twistAngular.z = angular*0.75f + angularPrev*0.25f;
	
	twist.linear = twistLinear;
	twist.angular = twistAngular;
	
	vel_pub_.publish(twist);
	
	//printf("linear: %f angular: %f \n", twistAngular.z, twistLinear.x);
}

// Main
int main(int argc, char** argv)
{
  ros::init(argc, argv, "create_teleop");

  CreateTeleop myCreateTeleop;


  ros::spin();
}
