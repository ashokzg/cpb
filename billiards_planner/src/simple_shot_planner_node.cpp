
#include<stdio.h>
#include<stdlib.h>

#include <ros/ros.h>

#include <billiards_planner/simple_shot_planner.h>

int main(int argc, char **argv)
{
  // start a ros node
  ros::init(argc,argv,"simple_shot_planner",ros::init_options::NoSigintHandler);
  // start a node
  simple_shot_planner::SimpleShotPlanner billiards_planner;

  ros::spin(); // don't need spin, but have it here for now.

  return 0;
}
