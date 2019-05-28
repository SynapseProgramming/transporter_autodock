#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <transporter_autodock/autodockAction.h>
#include <iostream>
#include <std_msgs/Bool.h>

typedef actionlib::SimpleActionClient<transporter_autodock::autodockAction> dock_ac;


class cdock_client{
public:
cdock_client():
ac("charging_dock_server",true),
sub(n.subscribe<std_msgs::Bool>("goto_dock",5,&cdock_client::dock_signal,this))
{
  std::cout<<"cdock controller client initialised\nWaiting for move_base server to start!"<<std::endl;
  ac.waitForServer();
  std::cout<<"cdock server is online!"<<std::endl;
}

void dock_signal(const std_msgs::Bool::ConstPtr &dock){
if((dock->data)==true&&going_to_dock==false){
  transporter_autodock::autodockGoal goal;
  goal.dock_goal=true;
  going_to_dock=true;
  std::cout<<"sending goal to cdock controller!"<<std::endl;
  ac.sendGoal(goal,boost::bind(&cdock_client::donecb, this, _1, _2),dock_ac::SimpleActiveCallback(), dock_ac::SimpleFeedbackCallback());
}
else{std::cout<<"Invalid command, gonna do nothing!"<<std::endl;}
}

void donecb(const actionlib::SimpleClientGoalState &state, const transporter_autodock::autodockResultConstPtr &result){
//Print out the current state.
std::cout<<"Status:"<<state.toString()<<std::endl;
//Statements to print upon sucessfully reaching the goal.
if(state.toString()=="SUCCEEDED"){std::cout<<"The goal has been reached!"<<std::endl;}
else{std::cout<<"Something went wrong! the bot could not reach the near goal!"<<std::endl;}
going_to_dock=false;
}//bracket of donecb


private:
ros::NodeHandle n;
ros::Subscriber sub;

bool going_to_dock=false;
dock_ac ac;
};


int main(int argc, char **argv){
ros::init(argc,argv,"cdock_buttons");

cdock_client client;


ros::spin();
return 0;}
