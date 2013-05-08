#include <stdio.h>
#include <ros/ros.h>

#include<roslib/Header.h>
#include<billiards_msgs/BallState.h>
#include<billiards_msgs/TableState.h>
#include<billiards_msgs/ShotPlan.h>
#include<billiards_planner/PlanOneShot.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <billiards_msgs/PlanShotAction.h>
#include <billiards_msgs/Constants.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

bool check_action(billiards_msgs::TableState ts,double angle_min,double angle_max)
{
  actionlib::SimpleActionClient<billiards_msgs::PlanShotAction> action_client("plan_shot", true);
  action_client.waitForServer();

  ROS_INFO("plan_shot action server is up, sending goal");
  billiards_msgs::PlanShotGoal goal;
  goal.state = ts;
  goal.angle_min = angle_min;
  goal.angle_max = angle_max;
  action_client.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = action_client.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = action_client.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    return (state == actionlib::SimpleClientGoalState::SUCCEEDED);
  }
  else
  {
    ROS_INFO("Action did not finish before the time out.");
    return false;
  }

}

void test_straight_shot()
{
  double table_length = billiards_msgs::Constants::TABLE_LENGTH;
  double table_width = billiards_msgs::Constants::TABLE_WIDTH;
  double radius = billiards_msgs::Constants::BALL_RADIUS;

  billiards_msgs::TableState ts;
  billiards_msgs::BallState bs;
  ts.balls.clear();
  // cue ball
  bs.id = 0;
  bs.pocketed = false;
  bs.point.header.stamp = ros::Time::now();
  bs.point.header.frame_id = "table";
  bs.point.point.x = table_length/4.0; // center of the table
  bs.point.point.y = table_width/4.0;
  bs.point.point.z = radius;
  ts.balls.push_back(bs);
  // ball 1
  bs.id = 1;
  bs.pocketed = false;
  bs.point.header.stamp = ros::Time::now();
  bs.point.header.frame_id = "table";
  bs.point.point.x = table_length/2.0;
  bs.point.point.y = table_width/2.0;
  bs.point.point.z = radius;
  ts.balls.push_back(bs);

  if (check_action(ts,-M_PI,M_PI)==true) ROS_INFO("test_straight_shot ok");
  else ROS_INFO("test_straight_shot not ok");
}

void test_along_rail()
{
  double table_length = billiards_msgs::Constants::TABLE_LENGTH;
  double table_width = billiards_msgs::Constants::TABLE_WIDTH;
  double radius = billiards_msgs::Constants::BALL_RADIUS;
  double rail = 0;//billiards_msgs::Constants::RAIL_DEPTH;

  billiards_msgs::TableState ts;
  billiards_msgs::BallState bs;
  ts.balls.clear();
  // cue ball
  bs.id = 0;
  bs.pocketed = false;
  bs.point.header.stamp = ros::Time::now();
  bs.point.header.frame_id = "table";
  bs.point.point.x = table_length-4*radius-rail;
  bs.point.point.y = table_width-radius-rail;
  bs.point.point.z = radius;
  ts.balls.push_back(bs);
  // ball 5
  bs.id = 5;
  bs.pocketed = false;
  bs.point.header.stamp = ros::Time::now();
  bs.point.header.frame_id = "table";
  bs.point.point.x = table_length/4;
  bs.point.point.y = table_width-radius-rail;
  bs.point.point.z = radius;
  ts.balls.push_back(bs);
  // ball 6
  bs.id = 6;
  bs.pocketed = false;
  bs.point.header.stamp = ros::Time::now();
  bs.point.header.frame_id = "table";
  bs.point.point.x = table_length-2.0*radius-rail;
  bs.point.point.y = table_width-radius-rail;
  bs.point.point.z = radius;
  ts.balls.push_back(bs);

  if (check_action(ts,-M_PI,M_PI)==true) ROS_INFO("test along rail ok");
  else ROS_INFO("test along rail not ok");
}

void test_unreachable()
{
  double table_width = billiards_msgs::Constants::TABLE_WIDTH;
  double radius = billiards_msgs::Constants::BALL_RADIUS;
  double rail = 0;//billiards_msgs::Constants::RAIL_DEPTH;

  billiards_msgs::TableState ts;
  billiards_msgs::BallState bs;
  ts.balls.clear();
  // cue ball
  bs.id = 0;
  bs.pocketed = false;
  bs.point.header.stamp = ros::Time::now();
  bs.point.header.frame_id = "table";
  bs.point.point.x = 4*radius+rail;
  bs.point.point.y = table_width/2.0;
  bs.point.point.z = radius;
  ts.balls.push_back(bs);
  // ball 5
  bs.id = 5;
  bs.pocketed = false;
  bs.point.header.stamp = ros::Time::now();
  bs.point.header.frame_id = "table";
  bs.point.point.x = 2.0*radius+rail;
  bs.point.point.y = table_width/2.0;
  bs.point.point.z = radius;
  ts.balls.push_back(bs);

  if (check_action(ts,-M_PI,M_PI)==false) ROS_INFO("test unreachable ok");
  else ROS_INFO("test unreachable not ok");
}

void test_too_thin()
{
  double table_length = billiards_msgs::Constants::TABLE_LENGTH;
  double table_width = billiards_msgs::Constants::TABLE_WIDTH;
  double radius = billiards_msgs::Constants::BALL_RADIUS;
  double rail = 0;//billiards_msgs::Constants::RAIL_DEPTH;

  billiards_msgs::TableState ts;
  billiards_msgs::BallState bs;
  ts.balls.clear();
  // cue ball
  bs.id = 0;
  bs.pocketed = false;
  bs.point.header.stamp = ros::Time::now();
  bs.point.header.frame_id = "table";
  bs.point.point.x = table_length - radius-rail;
  bs.point.point.y = table_width/2.0;
  bs.point.point.z = radius;
  ts.balls.push_back(bs);
  // ball 5
  bs.id = 5;
  bs.pocketed = false;
  bs.point.header.stamp = ros::Time::now();
  bs.point.header.frame_id = "table";
  bs.point.point.x = radius+rail;
  bs.point.point.y = table_width/2.0-2.0*radius;
  bs.point.point.z = radius;
  ts.balls.push_back(bs);

  if (check_action(ts,-M_PI,M_PI)==false) ROS_INFO("test too thin ok");
  else ROS_INFO("test too thin not ok");
}

void test_obstructed()
{
  double table_length = billiards_msgs::Constants::TABLE_LENGTH;
  double table_width = billiards_msgs::Constants::TABLE_WIDTH;
  double radius = billiards_msgs::Constants::BALL_RADIUS;
  double rail = 0;//billiards_msgs::Constants::RAIL_DEPTH;

  billiards_msgs::TableState ts;
  billiards_msgs::BallState bs;
  ts.balls.clear();
  // cue ball
  bs.id = 0;
  bs.pocketed = false;
  bs.point.header.stamp = ros::Time::now();
  bs.point.header.frame_id = "table";
  bs.point.point.x = 2.0*radius + rail;
  bs.point.point.y = 2.0*radius + rail;
  bs.point.point.z = radius;
  ts.balls.push_back(bs);
  // ball 5
  bs.id = 5;
  bs.pocketed = false;
  bs.point.header.stamp = ros::Time::now();
  bs.point.header.frame_id = "table";
  bs.point.point.x = table_length - 2.0*radius - rail;
  bs.point.point.y = table_width - 2.0*radius - rail;
  bs.point.point.z = radius;
  ts.balls.push_back(bs);
  // ball 8
  bs.id = 8;
  bs.pocketed = false;
  bs.point.header.stamp = ros::Time::now();
  bs.point.header.frame_id = "table";
  bs.point.point.x = table_length - 4.0*radius - rail;
  bs.point.point.y = table_width - 4.0*radius - rail;
  bs.point.point.z = radius;
  ts.balls.push_back(bs);

  if (check_action(ts,-M_PI,M_PI)==false) ROS_INFO("test_obstructed ok");
  else ROS_INFO("test_obstructed not ok");
}

void usage()
{
  printf("usage: test_planner [straight|rail|unreachable|thin|obstructed]\n");
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_shot_action_client");
  ros::NodeHandle n;

  ros::MultiThreadedSpinner s(2);
  boost::thread spinner_thread( boost::bind( &ros::spin, s ) );

  if (argc > 1)
  {
    if (strcmp(argv[1],"straight")==0) test_straight_shot();
    else if (strcmp(argv[1],"rail")==0) test_along_rail();
    else if (strcmp(argv[1],"unreachable")==0) test_unreachable();
    else if (strcmp(argv[1],"thin")==0) test_too_thin();
    else if (strcmp(argv[1],"obstructed")==0) test_obstructed();
    else usage();
  }
  else usage();

}


