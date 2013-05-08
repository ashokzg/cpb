#include <ros/ros.h>

#include<std_msgs/Header.h>
#include<billiards_msgs/BallState.h>
#include<billiards_msgs/TableState.h>
#include<billiards_msgs/ShotPlan.h>
#include<billiards_planner/PlanOneShot.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_table_state_broadcaster");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<billiards_msgs::TableState>("table_state", 100);
  ros::Rate loop_rate(5);
  int count = 0;

  double table_length = 2.4384;  // approximate
  double table_width = 1.2192;  // approximate

  billiards_msgs::TableState ts; ts.balls.clear();
  billiards_msgs::BallState bs;
  // cue ball
  bs.id = 0;
  bs.pocketed = false;
  bs.point.header.stamp = ros::Time::now();
  bs.point.header.frame_id = "table";
  bs.point.point.x = table_length/2; // center of the table
  bs.point.point.y = table_width/2;
  bs.point.point.z = 0;
  ts.balls.push_back(bs);
  // ball 1
  bs.id = 1;
  bs.pocketed = false;
  bs.point.header.stamp = ros::Time::now();
  bs.point.header.frame_id = "table";
  bs.point.point.x = table_length/2;
  bs.point.point.y = table_width/2 - 0.1;
  bs.point.point.z = 0;
  ts.balls.push_back(bs);
  // ball 2
  bs.id = 2;
  bs.pocketed = false;
  bs.point.header.stamp = ros::Time::now();
  bs.point.header.frame_id = "table";
  bs.point.point.x = table_length/2;
  bs.point.point.y = table_width/2 + 0.1;
  bs.point.point.z = 0;
  ts.balls.push_back(bs);
  // ball 3
  bs.id = 3;
  bs.pocketed = false;
  bs.point.header.stamp = ros::Time::now();
  bs.point.header.frame_id = "table";
  bs.point.point.x = table_length/2;
  bs.point.point.y = table_width/2 + 0.2;
  bs.point.point.z = 0;
  ts.balls.push_back(bs);

  while (ros::ok())
  {
    pub.publish(ts);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
}
