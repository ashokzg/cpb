#include <ros/ros.h>
#include <string>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include "simple_trajectory/MoveJoint.h"
#include "simple_trajectory/MoveMultipleJoints.h"
//#include "joint_states_listener/ReturnJointStates.h"
#include "/home/teamb/cpb/joint_states_listener/srv_gen/cpp/include/joint_states_listener/ReturnJointStates.h"

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action
  // used to trigger the arm movement action
  TrajClient* traj_client_right;
  TrajClient* traj_client_left;
  //our goal variable
  pr2_controllers_msgs::JointTrajectoryGoal r_goal, l_goal;
  ros::ServiceClient curJointAngles_client;
  typedef enum
  {
	  JOINT_POSITION = 0,
	  JOINT_VELOCITY,
	  JOINT_EFFORT,
	  NO_OF_JOINT_PARAMETERS
  } jointParameters_t;
public:
  typedef enum
  {
	  RIGHT_ARM = 0,
	  LEFT_ARM,
	  NO_OF_ARMS
  } armSide_t;

  //! Initialize the action client and wait for action server to come up
  RobotArm(ros::NodeHandle n)
  {
    // tell the action client that we want to spin a thread by default
    traj_client_right = new TrajClient("r_arm_controller/joint_trajectory_action", true);
    // tell the action client that we want to spin a thread by default
    traj_client_left = new TrajClient("l_arm_controller/joint_trajectory_action", true);
    // wait for action server to come up
    while((!traj_client_right->waitForServer(ros::Duration(5.0)) || (!traj_client_left->waitForServer(ros::Duration(5.0))))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
    addJointNames();
    curJointAngles_client = n.serviceClient<joint_states_listener::ReturnJointStates>("return_joint_states");
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_right;
  }

  void addJointNames()
  {
		// First, the joint names, which apply to all waypoints
		r_goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
		r_goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
		r_goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
		r_goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
		r_goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
		r_goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
		r_goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
	    // First, the joint names, which apply to all waypoints
	    l_goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
	    l_goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
	    l_goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
	    l_goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
	    l_goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
	    l_goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
	    l_goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(armSide_t side, pr2_controllers_msgs::JointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    if(side == RIGHT_ARM)
    	traj_client_right->sendGoal(goal);
    else
    	traj_client_left->sendGoal(goal);
    // Wait for trajectory completion
    while(!getState(side).isDone() && ros::ok())
    {
      usleep(50000);
    }
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory()
  {

    // We will have two waypoints in this goal trajectory
    r_goal.trajectory.points.resize(2);

    // First trajectory point
    // Positions
    int ind = 0;
    r_goal.trajectory.points[ind].positions.resize(7);
    r_goal.trajectory.points[ind].positions[0] = 0.0;
    r_goal.trajectory.points[ind].positions[1] = 0.0;
    r_goal.trajectory.points[ind].positions[2] = 0.0;
    r_goal.trajectory.points[ind].positions[3] = 0.0;
    r_goal.trajectory.points[ind].positions[4] = 0.0;
    r_goal.trajectory.points[ind].positions[5] = 0.0;
    r_goal.trajectory.points[ind].positions[6] = 0.0;
    // Velocities
    r_goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      r_goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    r_goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    // Second trajectory point
    // Positions
    ind += 1;
    r_goal.trajectory.points[ind].positions.resize(7);
    r_goal.trajectory.points[ind].positions[0] = -0.3;
    r_goal.trajectory.points[ind].positions[1] = 0.2;
    r_goal.trajectory.points[ind].positions[2] = -0.1;
    r_goal.trajectory.points[ind].positions[3] = -1.2;
    r_goal.trajectory.points[ind].positions[4] = 1.5;
    r_goal.trajectory.points[ind].positions[5] = -0.3;
    r_goal.trajectory.points[ind].positions[6] = 0.5;
    // Velocities
    r_goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      r_goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    r_goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

    //we are done; return the goal
    return r_goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState(armSide_t side)
  {
	  if(side == RIGHT_ARM)
		  return traj_client_right->getState();
	  else
		  return traj_client_left->getState();
  }

  void updateGoal(pr2_controllers_msgs::JointTrajectoryGoal& goal, int goalIdx, float angle, float vel)
  {
	  goalIdx %= 7;
	  goal.trajectory.points[0].positions[goalIdx] = angle;
  }

  bool getCurrentJointVales(armSide_t side, float (&jointParameters)[NO_OF_JOINT_PARAMETERS][7])
  {
	  joint_states_listener::ReturnJointStates jointStates;
	  jointStates.request.name.resize(7);
	  std::string jointNames[] = {"x_shoulder_pan_joint",
			  	  	  	  	  	  "x_shoulder_lift_joint",
								  "x_upper_arm_roll_joint",
								  "x_elbow_flex_joint",
								  "x_forearm_roll_joint",
								  "x_wrist_flex_joint",
								  "x_wrist_roll_joint"};
	  ROS_INFO("1");
	  // Change the joint names as per the side
	  for(int i = 0; i < 7; i++)
	  {
		  if(side == RIGHT_ARM)
			  jointNames[i].replace(0, 2, "r_");
		  else
			  jointNames[i].replace(0, 2, "l_");
		  jointStates.request.name[i] = jointNames[i];
	  }

	  ROS_INFO("Camer");
	  if(curJointAngles_client.call(jointStates))
	  {
		  for(int i = 0; i < 7; i++)
		  {
			  ROS_INFO("Joint %d: %f", i, jointStates.response.position[i]);
			  jointParameters[JOINT_POSITION][i] = jointStates.response.position[i];
			  jointParameters[JOINT_VELOCITY][i] = jointStates.response.velocity[i];
			  jointParameters[JOINT_EFFORT][i] = jointStates.response.effort[i];
		  }
		  return 0;
	  }
	  else
	  {
		  ROS_ERROR("Failed to get current joint angles");
		  return 1;
	  }
  }

  bool moveJoint(simple_trajectory::MoveJoint::Request  &req,
           simple_trajectory::MoveJoint::Response &res)
  {
      ROS_INFO("Moving one joint %d of arm %d by %f degrees with %f velocity", req.jointIdx, req.arm, req.movAngle, req.movVel);

      //Stores the current joint values
      // 0: Joint angles
      // 1: Joint velocities
      // 2: Joint efforts
      float curJointValues[NO_OF_JOINT_PARAMETERS][7];
      //Limiting jointIdx to 7 joints
      int jointIdx = req.jointIdx%7;
      //Limiting arm to boolean. O for right, 1 for left
      armSide_t side = static_cast<armSide_t>(req.arm%2);
      pr2_controllers_msgs::JointTrajectoryGoal& goalRef = (side == RIGHT_ARM)? r_goal: l_goal;
      //Resize the response
      res.curPosition.resize(7);

      //Get the current joint values
      try
      {
    	  if(getCurrentJointVales(side, curJointValues) == 1)
    		  throw "Unable to get joint limits";
      }
      catch(char *str)
      {
    	  ROS_ERROR("%s", str);
      }

      // We will have two waypoints in this goal trajectory
      goalRef.trajectory.points.resize(1);
      ROS_INFO("Got the joint values");
      // First trajectory point
      // Positions
      int ind = 0;
      // Resize position and velocities
      goalRef.trajectory.points[ind].positions.resize(7);
      goalRef.trajectory.points[ind].velocities.resize(7);
      //Initialize the position and velocity to current values so that we dont change any other joint
      for (size_t j = 0; j < 7; ++j)
      {
    	goalRef.trajectory.points[ind].positions[j] = curJointValues[JOINT_POSITION][j];
        goalRef.trajectory.points[ind].velocities[j] = curJointValues[JOINT_VELOCITY][j];
      }

      //Update the relevant joint with the user given values
      //WARNING: No limit checking here
      goalRef.trajectory.points[ind].positions[jointIdx] = req.movAngle;
      goalRef.trajectory.points[ind].velocities[jointIdx] = req.movVel;
      ROS_INFO("All set to move");
      // To be reached 1 second after starting along the trajectory
      goalRef.trajectory.points[ind].time_from_start = ros::Duration(1.0);

      startTrajectory(side, goalRef);
      //Get the current joint values
      try
	  {
	    if(getCurrentJointVales(side, curJointValues) == 1)
		    throw "Unable to get joint limits";
	  }
	  catch(char *str)
	  {
  	    ROS_ERROR("%s", str);
	  }
	  for(int i = 0; i < 7; i++)
		  res.curPosition[i] = curJointValues[JOINT_POSITION][i];
      //we are done with service
      return true;
  }

  bool  moveMultJoints(simple_trajectory::MoveMultipleJoints::Request  &req,
                       simple_trajectory::MoveMultipleJoints::Response &res)
  {
      //ROS_INFO("Moving one joint %d of arm %d by %f degrees with %f velocity", req.jointIdx, req.arm, req.movAngle, req.movVel);

      //Limiting arm to boolean. O for right, 1 for left
      armSide_t side = static_cast<armSide_t>(req.arm%2);
      pr2_controllers_msgs::JointTrajectoryGoal& goalRef = (side == RIGHT_ARM)? r_goal: l_goal;

      res.success = 1;

      // We will have two waypoints in this goal trajectory
      goalRef.trajectory.points.resize(1);
      ROS_INFO("Got the joint values");
      // First trajectory point
      // Positions
      int ind = 0;
      // Resize position and velocities
      goalRef.trajectory.points[ind].positions.resize(7);
      goalRef.trajectory.points[ind].velocities.resize(7);
      //Initialize the position and velocity to current values so that we dont change any other joint
      for (size_t j = 0; j < 7; ++j)
      {
    	goalRef.trajectory.points[ind].positions[j] = req.jointAngles[j];
        goalRef.trajectory.points[ind].velocities[j] = 0.5;
      }

      ROS_INFO("All set to move");
      // To be reached 1 second after starting along the trajectory
      goalRef.trajectory.points[ind].time_from_start = ros::Duration(1.0);

      startTrajectory(side, goalRef);

      res.success = 1;
      //we are done with service
      return true;
  }
};



int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "simple_trajectory");
  ros::NodeHandle n;

  RobotArm arm(n);
  ros::ServiceServer jnt = n.advertiseService("move_one_joint", &RobotArm::moveJoint, &arm);
  ros::ServiceServer mjnt = n.advertiseService("move_multiple_joints", &RobotArm::moveMultJoints, &arm);


  // Start the trajectory
  //arm.startTrajectory(RobotArm::RIGHT_ARM, arm.armExtensionTrajectory());
  ros::spin();
  return 0;

}
