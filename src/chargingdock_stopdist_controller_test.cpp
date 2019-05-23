#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <cmath>

double m_dist=0;

void distancecb(const std_msgs::Float32::ConstPtr &dist){
//update m_dist in (m)
m_dist=dist->data;
}

int main(int argc, char**argv){
ros::init(argc,argv,"dist_p_controller_test");
ros::NodeHandle n;
ros::Rate loop_rate(60);//we would want to run the loop at 60 hz
ros::Subscriber sub=n.subscribe("dock_dist",10,distancecb);
//we will send velocity commands to the mobile base
ros::Publisher pub=n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",20);

double Kp=0.2;
double e=0;
double e_abs=0;
double set_dist=0.5;
double c_signal=0;
const double lower_speed_threshold=0.03;
const double upper_speed_threshold=0.08;
const double stop_threshold=0.05;

//create the twist variable
geometry_msgs::Twist cmd_vel;

while(ros::ok()){

//calculate error
e=m_dist-set_dist;
//calculate absolute error
e_abs=std::abs(e*1.0);
//calculate control signal control signal
c_signal=Kp*e;
//Limit the control signal
if(c_signal<=lower_speed_threshold){c_signal=lower_speed_threshold;}
else if(c_signal>=upper_speed_threshold){c_signal=upper_speed_threshold;}
//stop the bot if it has reached the desired distance from goal
if(e_abs<=stop_threshold){c_signal=0;std::cout<<"The desired distance has been reached!"<<std::endl; break; }
std::cout<<"The absolute error is: "<<e_abs<<" The control signal is: "<<c_signal<<std::endl;


//populate the twist variable with stuff
cmd_vel.linear.x=c_signal;
//we want to ensure that everything else is 0
cmd_vel.linear.y=0;
cmd_vel.linear.z=0;
cmd_vel.angular.x=0;
cmd_vel.angular.y=0;
cmd_vel.angular.z=0;

//publish the velocity command
pub.publish(cmd_vel);

loop_rate.sleep();
ros::spinOnce();
}

return 0;
}
