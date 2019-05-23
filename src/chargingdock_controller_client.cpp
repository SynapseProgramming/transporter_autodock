#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <transporter_autodock/autodockAction.h>
#include <iostream>

//function called when goal reached!
void donecb(const actionlib::SimpleClientGoalState& state, const transporter_autodock::autodockResultConstPtr &result){
std::cout<<"autodock status: "<<state.toString()<< std::endl;
}
//called when action server just received a goal!
void activecb(){
std::cout<<"Moving to dock!"<<std::endl;
}

void feedbackcb(const transporter_autodock::autodockFeedbackConstPtr &feedback){
if((feedback->state)==1){std::cout<<"moving to far goal!"<<std::endl;}
else if((feedback->state)==2){std::cout<<"moving to near goal!"<<std::endl;}
else if((feedback->state)==3){std::cout<<"moving towards dock!"<<std::endl;}
else if((feedback->state)==4){std::cout<<"Reached Dock!"<<std::endl;}
}

int main(int argc, char **argv){
ros::init(argc, argv,"dock_controller_client");

actionlib::SimpleActionClient<transporter_autodock::autodockAction>ac("charging_dock_server",true);
std::cout<<"Waiting for the dock server to start! "<<std::endl;

ac.waitForServer();
std::cout<<"The server has started!"<<std::endl;
int input=0;
bool goal_state=false;

while(ros::ok()){
transporter_autodock::autodockGoal sent_goal;

std::cout<<"Press and enter 1 to send a dock goal start signal! "<<std::endl;
std::cin>>input;

//set the dock goal as true
if(input==1){
goal_state=true;
}
else{goal_state=false;
std::cout<<"enter 1 to send a goal start signal! "<<std::endl;
}

sent_goal.dock_goal=goal_state;
ac.sendGoal(sent_goal, &donecb, &activecb,&feedbackcb);

ac.waitForResult();


}//while loop bracket




return 0;
}
