#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <simple_gripper/ChangeGripper.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper{
private:
  GripperClient* gripper_r;
  GripperClient* gripper_l;
  GripperClient* gripper_client_ ;  

public:
  //Action client initialization
  Gripper(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_l = new GripperClient("l_gripper_controller/gripper_action", true);
    gripper_r = new GripperClient("r_gripper_controller/gripper_action", true);
    gripper_client_ = gripper_r;
    //gripper_client_ = new GripperClient("l_gripper_controller/gripper_action", true);
    
    //wait for the gripper action server to come up 
    while(!gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the l_gripper_controller/gripper_action action server to come up");
    }
  }

  ~Gripper(){
    delete gripper_client_;
  }

  //Open the gripper
  void open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.08;
    open.command.max_effort = -1.0;  // Do not limit effort (negative)
    
    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper opened!");
    else
      ROS_INFO("The gripper failed to open.");
  }

  //Close the gripper
  void close(){
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.0;
    squeeze.command.max_effort = 50.0;  // Close gently
    
    ROS_INFO("Sending squeeze goal");
    gripper_client_->sendGoal(squeeze);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper closed!");
    else
      ROS_INFO("The gripper failed to close.");
  }
    
  bool changeGrip(simple_gripper::ChangeGripper::Request  &req,
           simple_gripper::ChangeGripper::Response &res)
  {
    ROS_INFO("Camer");
    if(req.arm == 0)
         gripper_client_ = gripper_r;
    else
         gripper_client_ = gripper_l;        
    if(req.openClose == 0)
        open();
    else
        close();    
    res.success = true;
    return true;
  }
  
};

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_gripper");

  Gripper gripper;

  // Init the ROS node
  ros::NodeHandle n;

  ros::ServiceServer grip = n.advertiseService("gripper", &Gripper::changeGrip, &gripper);

  ros::spin();

  return 0;
}