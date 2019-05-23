#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <transporter_autodock/autodockAction.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> movebAC;

class get_transform{
public:

void obtain_transform(const std::string &parent_frame,const std::string &child_frame, double &x, double &y, double &z, double &w){
  tf::StampedTransform received_transform;

  try{
  //lookupTransform(parent frame,child frame) gets transform from parent frame to child frame.
  listener.lookupTransform(parent_frame,child_frame,ros::Time(0),received_transform);

  }
  //if an exception has occured, print out what went wrong
  catch(tf::TransformException &ex){
  //ROS_ERROR("%s",ex.what());
  //ros::Duration(1.0).sleep();
  }
  //if nothing went wrong(yay), we would obtain the transform data
  //over here, we would get the xy transforms
  x=received_transform.getOrigin().x();
  y=received_transform.getOrigin().y();

  //over here, we would get the quaterion z w values.
  z=received_transform.getRotation().z();
  w=received_transform.getRotation().w();

  //actually print out the text
  //std::cout<<"From: "<<parent_frame<<" To "<<child_frame<<"X:"<<tx<<" Y:"<<ty<<" Z:"<<tz<<std::endl;

  }

private:
tf::TransformListener listener;

};



class move_base_controller{
public:
move_base_controller():
ac("move_base",true)
{
std::cout<<"move base controller initialised!\nWaiting for move_base server to start!"<<std::endl;
ac.waitForServer();
std::cout<<"move base server is online!"<<std::endl;
}

void movebase_send_goal(double x, double y, double z,double w){
  //create the move base goal variable.
  move_base_msgs::MoveBaseGoal goal;
  //populate the goal with data.
  goal.target_pose.header.frame_id="map";
  goal.target_pose.header.stamp=ros::Time::now();

  goal.target_pose.pose.position.x=x;
  goal.target_pose.pose.position.y=y;
  goal.target_pose.pose.orientation.w=w;
  goal.target_pose.pose.orientation.z=z;

  //send the goal to move base!
  ac.sendGoal(goal,boost::bind(&move_base_controller::donecb, this, _1, _2),movebAC::SimpleActiveCallback(), movebAC::SimpleFeedbackCallback());
  g_reached=false;
}//bracket of movebase_send_goal

void donecb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result){
//Print out the current state.
std::cout<<"Status:"<<state.toString()<<std::endl;
//Statements to print upon sucessfully reaching the goal.
if(state.toString()=="SUCCEEDED"){
std::cout<<"The goal has been reached!"<<std::endl;
//declare goal reached as true.
g_reached=true;
}
else{g_reached=false;
std::cout<<"Something went wrong! the bot could not reach the near goal!"<<std::endl;
}

}//bracket of donecb

bool get_goal_state(){
return g_reached;
}

private:
//create the action client object,specifying the name of the action server.
movebAC ac;
bool g_reached=false;

};


class cdock_controller{
public:
cdock_controller():
as_(n_,"charging_dock_server",boost::bind(&cdock_controller::executecb,this,_1),false),
d_sub(n_.subscribe<std_msgs::Float32>("dock_dist",10,&cdock_controller::distancecb,this)),
d_pub(n_.advertise<geometry_msgs::Twist>("dock_cmd_vel",20))
{
as_.start();

if(
n_.getParam("Kp",Kp)!=true
||n_.getParam("stop_distance",set_dist)!=true
||n_.getParam("lower_speed_threshold",lower_speed_threshold)!=true
||n_.getParam("upper_speed_threshold",upper_speed_threshold)!=true
||n_.getParam("stop_threshold",stop_threshold)!=true
||n_.getParam("far_goal_x",far_goal_x)!=true
||n_.getParam("far_goal_y",far_goal_y)!=true
||n_.getParam("far_goal_z",far_goal_z)!=true
||n_.getParam("far_goal_w",far_goal_w)!=true
){ROS_ERROR("yaml parameters could not be loaded! please check the yaml file!");}

std::cout<<"charging dock controller initiated! "<<std::endl;
}

void distancecb(const std_msgs::Float32::ConstPtr &dist){
//update m_dist in (m)
m_dist=dist->data;
}//bracket of distancecb

void executecb(const transporter_autodock::autodockGoalConstPtr &goal){
ros::Rate loop_rate(60);//we would want to run the loop at 60 hz
bool success=true;
while(true){// this while loop contains the high level logic
//send feedback to the action client
feedback_.state=state;
as_.publishFeedback(feedback_);

//if somehow a kill signal is sent, do this stuff
if(as_.isPreemptRequested()||!ros::ok()){
as_.setPreempted();
success=false;
break;
}
else if(state==0&&goal->dock_goal==true){// firstly, move to a pose which is close to the dock. (aka far goal)
state=1;
move_dock_far_goal();
}
else if(state==1&&move_base.get_goal_state()==true){//once the bot has reached the far goal, get the (dock_goal) goal pose, and send it to move base
std::cout<<"The far goal has been reached!"<<std::endl;
move_dock_near_goal();
state=2;
}
else if(state==2&&move_base.get_goal_state()==true){
std::cout<<"The near goal has been reached!"<<std::endl;
ros::Duration(2.0).sleep(); //we would want to come a complete stop before moving off again.
state=3;
}
else if(state==3){//once the bot has reached the near goal, we will move forward till we have reached a desired stopping distance.
move_into_dock();
if(dock_reached==true){state=4;}
}

else if(state==4){
std::cout<<"SUCCESSFULLY DOCKED!"<<std::endl;
result_.dock_result="SUCCEEDED!";
as_.setSucceeded(result_);
state=0;
dock_reached=false;
//we would want to exit the loop here.
break;
}




loop_rate.sleep();
}//while loop bracket


}//bracket of executecb

protected:
  void move_dock_far_goal(){
    //x,y,z,w
  move_base.movebase_send_goal(far_goal_x,far_goal_y,far_goal_z,far_goal_w);
  std::cout<<"currently moving to dock far goal!"<<std::endl;
  }

//void obtain_transform(const std::string &parent_frame,const std::string &child_frame, double &x, double &y, double &z, double &w){
 void move_dock_near_goal(){
  double x,y,z,w;
  std::cout<<"currently waiting for 5 seconds for readings to stabalise"<<std::endl;
  ros::Duration(5.0).sleep(); //delay for 5 seconds
  std::cout<<"currently obtaining the transformation from: "<<near_goal_parent_frame<<" to: "<<near_goal_child_frame<<std::endl;
  near_goal_obtain_position.obtain_transform(near_goal_parent_frame,near_goal_child_frame,x,y,z,w);
  std::cout<<"currently sending goal to move base!"<<std::endl;
  move_base.movebase_send_goal(x,y,z,w);

 }

void move_into_dock(){
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
  if(e_abs<=stop_threshold){c_signal=0;std::cout<<"The desired distance has been reached!"<<std::endl; dock_reached=true; }
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
  d_pub.publish(cmd_vel);
}


//create the ros nodehandle object
ros::NodeHandle n_;
//create the actionserver
actionlib::SimpleActionServer<transporter_autodock::autodockAction> as_;

//create the feedback and result variables
transporter_autodock::autodockFeedback feedback_;
transporter_autodock::autodockResult result_;

//create an instance of move base controller(to send commands to move base)
move_base_controller move_base;

//create an instance of the get transform object
get_transform near_goal_obtain_position;

//state variable is used to manage the order of operations
int state=0;

const std::string near_goal_parent_frame="map";
const std::string near_goal_child_frame="dock_goal";

//declare far goal positions
double far_goal_x;
double far_goal_y;
double far_goal_z;
double far_goal_w;

//declare variables which are needed for the proportional distance controller
ros::Subscriber d_sub;
ros::Publisher d_pub;
double m_dist=0;
double Kp;
double e=0;
double e_abs=0;
double set_dist;
double c_signal=0;
double lower_speed_threshold;
double upper_speed_threshold;
double stop_threshold;
//create the twist variable
geometry_msgs::Twist cmd_vel;

bool dock_reached=false;

};



int main(int argc, char** argv){
ros::init(argc, argv,"charging_dock_controller");

cdock_controller controller;



ros::spin();
return 0;}
