#include <ros/ros.h>

#include "FastFiz.h"
#include <fastfiz_msgs/SimulateShot.h>

using namespace Pool;

Table g_table;

TableState msgToFiz(const billiards_msgs::TableState& state)
{
  TableState out_state(g_table);
  for (size_t i = 0; i < state.balls.size(); ++i)
  {
    Ball b((Ball::Type)state.balls[i].id, state.balls[i].pocketed ? Ball::POCKETED_E : Ball::STATIONARY);
    b.setPos(Pool::Point(g_table.getWidth() - state.balls[i].point.point.y, g_table.getLength() - state.balls[i].point.point.x));
    out_state.setBall(b);
  }

  return out_state;
}

ShotParams msgToFiz(const fastfiz_msgs::ShotParams& shot)
{
  return ShotParams(shot.a, shot.b, shot.theta, fmodf(270.0 - shot.phi, 360), shot.v);
}

billiards_msgs::BallState fizToMsg(const Ball& b)
{
  billiards_msgs::BallState out;
  out.id = b.getID();
  out.pocketed = b.isPocketed();
  out.point.point.x = g_table.getLength() - b.getPos().y;
  out.point.point.y = g_table.getWidth() - b.getPos().x;
  out.point.header.frame_id = "/table";
  out.point.header.stamp = ros::Time::now();
  return out;
}

billiards_msgs::TableState fizToMsg(const TableState& state)
{
  billiards_msgs::TableState out_state;

  std::vector<Ball>::const_iterator it = state.getBegin();
  std::vector<Ball>::const_iterator end = state.getEnd();
  for (; it != end; ++it)
  {
    const Ball& b = *it;
    out_state.balls.push_back(fizToMsg(b));
  }

  return out_state;
}

bool simulateShot(fastfiz_msgs::SimulateShotRequest& req, fastfiz_msgs::SimulateShotResponse& res)
{
  TableState ts = msgToFiz(req.state);
  ShotParams shot = msgToFiz(req.shot);
  Shot* s = ts.executeShot(shot, false, false);

  typedef std::vector<Event*> V_Event;
  const V_Event& events = s->getEventList();
  V_Event::const_iterator it = events.begin();
  V_Event::const_iterator end = events.end();
  for (; it != end; ++it)
  {
    Event* e = *it;
    fastfiz_msgs::Event event;
    event.type = e->getType();
    event.ball1 = fizToMsg(e->getBall1());
    event.ball2 = fizToMsg(e->getBall2());
    event.string_rep = e->toString();

    res.events.push_back(event);
  }

  delete s;

  res.state = fizToMsg(ts);

  return true;
}

void initTable()
{
  g_table = Table(2.235, 112.0, 0.125, 0.135);
  g_table.setRailHeight(0.038);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fastfiz_node");

  ros::NodeHandle nh;
  ros::ServiceServer simulate_shot_srv = nh.advertiseService("simulate_shot", simulateShot);

  ros::spin();
}
