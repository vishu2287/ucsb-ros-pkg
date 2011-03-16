#include <ros/ros.h>
#include <joy/Joy.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>

//=================================================================================================
//=================================================================================================
class CreateTeleop
{
  public:
    CreateTeleop();

  private:
    void JoyCallback(const joy::Joy::ConstPtr& joy);
    void SendTwist(const ros::TimerEvent& e);

    ros::NodeHandle nh;
    
    float linear, linearPrev;
    float angular, angularPrev;

    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    ros::Timer sendTimer;
};

//=================================================================================================
//=================================================================================================
CreateTeleop::CreateTeleop()
{
  linear      = 0.0f;
  linearPrev  = 0.0f;
  angular     = 0.0f;
  angularPrev = 0.0f;

  a_scale_ = 0.75;
  l_scale_ = 0.25;

  nh.param("scale_angular", a_scale_, a_scale_);
  nh.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh.advertise<geometry_msgs::Twist>("create_node/cmd_vel", 1);

  joy_sub_ = nh.subscribe<joy::Joy>("joy", 10, &CreateTeleop::JoyCallback, this);
  
  sendTimer = nh.createTimer(ros::Duration(0.1), &CreateTeleop::SendTwist, this);
  
}
//=================================================================================================
//=================================================================================================
void CreateTeleop::JoyCallback(const joy::Joy::ConstPtr& joy)
{
  linearPrev = linear;
  linear = l_scale_*joy->axes[1];
  angularPrev = angular;
  angular = a_scale_*joy->axes[0];
}

//=================================================================================================
//=================================================================================================
void CreateTeleop::SendTwist(const ros::TimerEvent& e)
{
  geometry_msgs::Twist twist;

  // Do some minor filtering as well
  twist.linear.x = linear*0.75f + linearPrev*0.25f;
  twist.angular.z = angular*0.75f + angularPrev*0.25f;

  vel_pub_.publish(twist);
}

//=================================================================================================
// Main
//=================================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "create_teleop");

  CreateTeleop myCreateTeleop;

  ros::spin();
}
