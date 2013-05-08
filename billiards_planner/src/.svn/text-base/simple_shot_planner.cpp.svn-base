
#include<stdio.h>
#include<stdlib.h>

#include <billiards_planner/simple_shot_planner.h>
#include <std_msgs/ColorRGBA.h>

using namespace simple_shot_planner;

SimpleShotPlanner::SimpleShotPlanner() : rosnode_(""), table_state_initialized_(true)
                         ,shot_angle_min_(-M_PI/2),shot_angle_max_(M_PI/2)
                         ,action_server_(rosnode_, "plan_shot")
{
  // start action server
  action_server_.registerGoalCallback(boost::bind(&SimpleShotPlanner::actionGoalCallback, this));
  action_server_.registerPreemptCallback(boost::bind(&SimpleShotPlanner::actionPreemptCallback, this));
  this->found_valid_shot_plan = false;
  this->best_score = 0;

  this->marker_pub_ = rosnode_.advertise<visualization_msgs::Marker>("planner_marker",0);

  /// \brief setup custom callback queue
  callback_queue_thread_ = new boost::thread( &SimpleShotPlanner::queueThread,this );

  this->ball_radius_ = billiards_msgs::Constants::BALL_RADIUS;

  this->bridge_distance_from_cue_ball_ = billiards_msgs::Constants::BRIDGE_TO_STRIKE_MIN;

  /// \brief assign pocket "goal" locations based on table size.
  double table_width = billiards_msgs::Constants::TABLE_WIDTH;
  double table_length = billiards_msgs::Constants::TABLE_LENGTH;
  double rail_depth = 0; //billiards_msgs::Constants::RAIL_DEPTH;
  double ball_radius = billiards_msgs::Constants::BALL_RADIUS;
  billiards_msgs::PocketState pocket_state;
  pocket_state.id = 0;
  pocket_state.point.header.frame_id = "table";
  pocket_state.point.point.x = rail_depth+ball_radius;
  pocket_state.point.point.y = rail_depth+ball_radius;
  pocket_state.point.point.z = ball_radius;
  this->pocket_states_.push_back(pocket_state);
  pocket_state.id = 1;
  pocket_state.point.header.frame_id = "table";
  pocket_state.point.point.x = table_length/2;
  pocket_state.point.point.y = rail_depth+ball_radius;
  pocket_state.point.point.z = ball_radius;
  this->pocket_states_.push_back(pocket_state);
  pocket_state.id = 2;
  pocket_state.point.header.frame_id = "table";
  pocket_state.point.point.x = table_length-rail_depth-ball_radius;
  pocket_state.point.point.y = rail_depth+ball_radius;
  pocket_state.point.point.z = ball_radius;
  this->pocket_states_.push_back(pocket_state);
  pocket_state.id = 3;
  pocket_state.point.header.frame_id = "table";
  pocket_state.point.point.x = table_length-rail_depth-ball_radius;
  pocket_state.point.point.y = table_width-rail_depth-ball_radius;
  pocket_state.point.point.z = ball_radius;
  this->pocket_states_.push_back(pocket_state);
  pocket_state.id = 4;
  pocket_state.point.header.frame_id = "table";
  pocket_state.point.point.x = table_length/2;
  pocket_state.point.point.y = table_width-rail_depth-ball_radius;
  pocket_state.point.point.z = ball_radius;
  this->pocket_states_.push_back(pocket_state);
  pocket_state.id = 5;
  pocket_state.point.header.frame_id = "table";
  pocket_state.point.point.x = rail_depth+ball_radius;
  pocket_state.point.point.y = table_width-rail_depth-ball_radius;
  pocket_state.point.point.z = ball_radius;
  this->pocket_states_.push_back(pocket_state);

  /// \brief advertise all services
  this->AdvertiseServices();
  this->SubscribeTopics();

}

SimpleShotPlanner::~SimpleShotPlanner()
{
  // shutdown ros
  this->rosnode_.shutdown();
  // shutdown ros queue
  this->callback_queue_thread_->join();
  delete this->callback_queue_thread_;
}

/// \brief plan shot action
void SimpleShotPlanner::actionGoalCallback()
{
  this->table_state_initialized_ = true;
  this->action_goal_ = action_server_.acceptNewGoal();
  this->table_state_ = this->action_goal_->state;
  this->shot_angle_min_ = this->action_goal_->angle_min;
  this->shot_angle_max_ = this->action_goal_->angle_max;
  billiards_planner::PlanOneShot::Request req;
  billiards_planner::PlanOneShot::Response res;
  if (planNextShot(req,res))
  {
    ROS_INFO("plan shot succeeded");
    this->action_result_.shot = this->best_shot_plan;
    this->action_server_.setSucceeded(this->action_result_);
  }
  else
  {
    ROS_INFO("plan shot aborted, no possible shots available");
    this->action_server_.setAborted(this->action_result_);
  }
}
/// \brief plan shot action
void SimpleShotPlanner::actionPreemptCallback()
{
  this->action_server_.setPreempted();
}

/// \brief ros queue thread for this node
void SimpleShotPlanner::queueThread()
{
  ROS_DEBUG_STREAM("Callback thread id=" << boost::this_thread::get_id());
  static const double timeout = 0.001;
  while (this->rosnode_.ok())
    this->queue_.callAvailable(ros::WallDuration(timeout));
}

/// \brief advertise services
void SimpleShotPlanner::AdvertiseServices()
{
  // Advertise spawn services on the custom queue
  std::string plan_next_shot_service_name("plan_next_shot");
  ros::AdvertiseServiceOptions plan_next_shot_aso = ros::AdvertiseServiceOptions::create<billiards_planner::PlanOneShot>(
      plan_next_shot_service_name,boost::bind(&SimpleShotPlanner::planNextShot,this,_1,_2),
      ros::VoidPtr(), &this->queue_);
  plan_next_shot_service_ = this->rosnode_.advertiseService(plan_next_shot_aso);
}

/// \brief subscribe to this topic
void SimpleShotPlanner::SubscribeTopics()
{
  // topic callback version for TableState
  ros::SubscribeOptions table_state_so = ros::SubscribeOptions::create<billiards_msgs::TableState>(
    "table_state",10, boost::bind( &SimpleShotPlanner::updateTableState,this,_1),
    ros::VoidPtr(), &this->queue_);
  table_state_listener_ = this->rosnode_.subscribe(table_state_so);
}

/// \brief update per topic
void SimpleShotPlanner::updateTableState(const billiards_msgs::TableState::ConstPtr& table_state)
{
  this->lock_.lock();
  if (!this->table_state_initialized_) table_state_initialized_ = true;
  this->table_state_ = *table_state; // make a copy of the table state
  this->lock_.unlock();
}

/// \brief plan next shot
bool SimpleShotPlanner::planNextShot(billiards_planner::PlanOneShot::Request &req,billiards_planner::PlanOneShot::Response &res)
{
  ros::Rate wait_loop_rate(1.0);
  while (!this->table_state_initialized_)
  {
    ROS_WARN("simple shot planner has not received any TableState messages (table_state topic), waiting...");
    wait_loop_rate.sleep();
  }

  // get cue ball position
  bool cue_ball_position_found = false;
  geometry_msgs::PointStamped cue_ball_position;
  for (std::vector<billiards_msgs::BallState>::iterator iter=this->table_state_.balls.begin();
       iter!=this->table_state_.balls.end(); iter++)
    if (req.cue_ball_id == (*iter).id)
    {
      geometry_msgs::PointStamped ball_position = iter->point;
      tf_listener_.transformPoint("table",ball_position,cue_ball_position);
      cue_ball_position_found = true;
    }
  if (!cue_ball_position_found)
  {
    ROS_ERROR("Cue ball id %d is not in TableState's list of balls",req.cue_ball_id);
    return false;
  }


  // loop through all targets, construct list of shot plans
  this->found_valid_shot_plan = false;
  this->best_score = -0.1;
  int best_pocket_id = 0;
  int best_ball_id = 0;
  res.shot_plans.clear();
  res.target_balls.clear();
  res.target_pockets.clear();
  res.scores.clear();
  for (std::vector<billiards_msgs::BallState>::iterator ball = this->table_state_.balls.begin();
       ball != this->table_state_.balls.end(); ball++)
  for (std::vector<billiards_msgs::PocketState>::iterator pocket = this->pocket_states_.begin();
       pocket != this->pocket_states_.end(); pocket++)
  if ((*ball).id != req.cue_ball_id)
  {
    res.target_balls.push_back(*ball);
    geometry_msgs::PointStamped ball_position = ball->point;
    geometry_msgs::PointStamped target_ball_position;
    tf_listener_.transformPoint("table",ball_position,target_ball_position);

    res.target_pockets.push_back(*pocket);
    geometry_msgs::PointStamped pocket_position = pocket->point;
    geometry_msgs::PointStamped target_pocket_position;
    tf_listener_.transformPoint("table",pocket_position,target_pocket_position);

    billiards_msgs::ShotPlan shot_plan;

    double score = planOneShot(cue_ball_position,target_ball_position,target_pocket_position,
                               req.cue_ball_id,(*ball).id,pocket->id,shot_plan);
    res.scores.push_back(score);
    res.shot_plans.push_back(shot_plan);

    // choose best plan
    if (score > this->best_score)
    {
      this->best_shot_plan = shot_plan;
      this->best_score = score;
      this->found_valid_shot_plan = true;
      best_ball_id = ball->id;
      best_pocket_id = pocket->id;
    }
  }
  ROS_INFO("Best Shot Ball %d into Pocket %d with Score %f",best_ball_id,best_pocket_id,this->best_score);

  // loop through plans
  if (this->found_valid_shot_plan)
  {
    ROS_INFO("service found valid shot plans");
    return true;
  }
  else
  {
    ROS_INFO("service did not find any valid shot plans");
    return false;
  }
}

/// \brief plan one shot
double SimpleShotPlanner::planOneShot(const geometry_msgs::PointStamped cue_ball_position
                                     ,const geometry_msgs::PointStamped target_ball_position
                                     ,const geometry_msgs::PointStamped target_pocket_position
                                     ,int cue_ball_id, int target_ball_id, int pocket_id
                                     ,billiards_msgs::ShotPlan& shot_plan)
{

  for (std::vector<billiards_msgs::BallState>::iterator ball = this->table_state_.balls.begin();                                       ball != this->table_state_.balls.end(); ball++)
  {
    visualization_msgs::Marker ball_marker;
    ball_marker.header.frame_id = "table";
    ball_marker.header.stamp = ros::Time::now();
    ball_marker.ns = "balls";
    ball_marker.id = ball->id;
    ball_marker.type = visualization_msgs::Marker::SPHERE;
    ball_marker.points.clear();
    ball_marker.colors.clear();
    ball_marker.scale.x = 2.0*this->ball_radius_;
    ball_marker.scale.y = 2.0*this->ball_radius_;
    ball_marker.scale.z = 2.0*this->ball_radius_;
    ball_marker.lifetime = ros::Duration();
    ball_marker.pose.position.x = ball->point.point.x;
    ball_marker.pose.position.y = ball->point.point.y;
    ball_marker.pose.position.z = ball->point.point.z;
    ball_marker.pose.orientation.x = 0.0;
    ball_marker.pose.orientation.y = 0.0;
    ball_marker.pose.orientation.z = 0.0;
    ball_marker.pose.orientation.w = 1.0;
    std_msgs::ColorRGBA c;
    if (ball->id == 0) c.r = c.g = c.b = c.a = 1.0f;
    else {c.r = c.g = c.a = 1.0f; c.b = 0.0f;}
    ball_marker.color = c;
    this->marker_pub_.publish(ball_marker);
  }

  // compute cue ball target position in the table frame
  //  this is the pose of the cue ball center at instant of contact with target ball.
  //  on contact, centers of the cue ball and the target ball should align with the pocket.
  // ROS_INFO("pocket: %f %f %f ball: %f %f %f"
  //                ,target_pocket_position.point.x ,target_pocket_position.point.y, target_pocket_position.point.z
  //                ,target_ball_position.point.x ,target_ball_position.point.y, target_ball_position.point.z);

  Eigen::Vector3d ball_to_pocket_direction( target_pocket_position.point.x - target_ball_position.point.x
                                           ,target_pocket_position.point.y - target_ball_position.point.y
                                           ,0); // throw away z info
  double ball_to_pocket_dist = ball_to_pocket_direction.norm();
  ball_to_pocket_direction.normalize();

  geometry_msgs::Point cue_ball_target_position;
  cue_ball_target_position.x = target_ball_position.point.x -2.0*this->ball_radius_ * ball_to_pocket_direction.x();
  cue_ball_target_position.y = target_ball_position.point.y -2.0*this->ball_radius_ * ball_to_pocket_direction.y();
  cue_ball_target_position.z = 0; // throw away z info

  // calculate target velocity direction vector in the table frame
  //  (vector from cue ball current position to contact position)
  Eigen::Vector3d shot_direction( cue_ball_target_position.x - cue_ball_position.point.x
                                 ,cue_ball_target_position.y - cue_ball_position.point.y
                                 ,0); // throw away z info
  double cue_to_ball_dist = shot_direction.norm();
  if (cue_to_ball_dist == 0) // cue ball is on target already, shoot towards target ball
    shot_direction = Eigen::Vector3d( target_ball_position.point.x - cue_ball_position.point.x
                                     ,target_ball_position.point.y - cue_ball_position.point.y
                                     ,0); // throw away z info
  shot_direction.normalize();

  ROS_DEBUG("cue ball: %f %f %f cue ball target: %f %f %f shot_direction: %f %f %f"
           ,cue_ball_position.point.x,cue_ball_position.point.y,cue_ball_position.point.z
           ,cue_ball_target_position.x,cue_ball_target_position.y,cue_ball_target_position.z
           ,shot_direction.x(),shot_direction.y(),shot_direction.z()
           );


  // compute bridge pose in the table frame
  //  bridge location is cue_ball_position - unit_shot_vector * ball radius
  //  remember that bridge pose +x points from cue tip to cue ball, and +z upwards.
  //    +y follows right hand rule.
  geometry_msgs::Pose bridge_pose;
  bridge_pose.position.x = cue_ball_position.point.x - this->bridge_distance_from_cue_ball_*shot_direction.x();
  bridge_pose.position.y = cue_ball_position.point.y - this->bridge_distance_from_cue_ball_*shot_direction.y();
  bridge_pose.position.z = 0; // bridge pose is fixed on the table surface, z perpendicular to table surface

  // get bridge orientation in table frame
  Eigen::Vector3d u = shot_direction;
  Eigen::Vector3d w (0.0, 0.0, 1.0); // z-up
  Eigen::Vector3d v = w.cross(u);
  Eigen::Matrix3d R; // temporarily store rotation matrix from table frame to bridge orientation
  R << u, v, w;
  Eigen::Quaternion<double> q (R);

  bridge_pose.orientation.x = q.coeffs()[0];
  bridge_pose.orientation.y = q.coeffs()[1];
  bridge_pose.orientation.z = q.coeffs()[2];
  bridge_pose.orientation.w = q.coeffs()[3];
  // fill in bridge pose
  shot_plan.bridge_pose.header = cue_ball_position.header;
  shot_plan.bridge_pose.pose = bridge_pose;

  this->bridge_pose_marker_.header.frame_id = "table";
  this->bridge_pose_marker_.header.stamp = ros::Time::now();
  this->bridge_pose_marker_.ns = "planner_bridge_pose";
  this->bridge_pose_marker_.id = target_ball_id+20*pocket_id;
  this->bridge_pose_marker_.type = visualization_msgs::Marker::ARROW;
  this->bridge_pose_marker_.pose.position.x    = bridge_pose.position.x;
  this->bridge_pose_marker_.pose.position.y    = bridge_pose.position.y;
  this->bridge_pose_marker_.pose.position.z    = bridge_pose.position.z;
  this->bridge_pose_marker_.pose.orientation.x = bridge_pose.orientation.x;
  this->bridge_pose_marker_.pose.orientation.y = bridge_pose.orientation.y;
  this->bridge_pose_marker_.pose.orientation.z = bridge_pose.orientation.z;
  this->bridge_pose_marker_.pose.orientation.w = bridge_pose.orientation.w;
  this->bridge_pose_marker_.scale.x = 0.03;
  this->bridge_pose_marker_.scale.y = 0.03;
  this->bridge_pose_marker_.scale.z = 0.03;
  this->bridge_pose_marker_.color.r = 1.0f;
  this->bridge_pose_marker_.color.g = 1.0f;
  this->bridge_pose_marker_.color.a = 1.0;
  this->bridge_pose_marker_.lifetime = ros::Duration();

  // compute pr2 base_pose
  // transform from r_gripper_tool_frame to base_link
  // taken from wiki by SG
  // At time 1275955442.985
  // - Translation: [-0.235, 0.237, -0.404]
  // - Rotation: in Quaternion [-0.066, -0.037, -0.038, 0.996]
  //             in RPY [-0.130, -0.078, -0.072]

  // bridge pose in table frame
  tf::Transform table_to_bridge(tf::Quaternion(bridge_pose.orientation.x
                                              ,bridge_pose.orientation.y
                                              ,bridge_pose.orientation.z
                                              ,bridge_pose.orientation.w)
                                  ,tf::Vector3(bridge_pose.position.x
                                              ,bridge_pose.position.y
                                              ,bridge_pose.position.z));

  // use constants
  tf::Transform base_to_bridge(tf::Quaternion(billiards_msgs::Constants::BRIDGE_IN_BASE_QX
                                             ,billiards_msgs::Constants::BRIDGE_IN_BASE_QY
                                             ,billiards_msgs::Constants::BRIDGE_IN_BASE_QZ
                                             ,billiards_msgs::Constants::BRIDGE_IN_BASE_QW),
                                  tf::Vector3(billiards_msgs::Constants::BRIDGE_IN_BASE_X
                                             ,billiards_msgs::Constants::BRIDGE_IN_BASE_Y
                                             ,billiards_msgs::Constants::BRIDGE_IN_BASE_Z));
  tf::Transform table_to_base_link = table_to_bridge * base_to_bridge.inverse();


  ROS_DEBUG("table_to_bridge:(%f %f %f)(%f %f %f %f)",table_to_bridge.getOrigin().x() 
                                                    ,table_to_bridge.getOrigin().y() 
                                                    ,table_to_bridge.getOrigin().z() 
                                                    ,table_to_bridge.getRotation().x() 
                                                    ,table_to_bridge.getRotation().y() 
                                                    ,table_to_bridge.getRotation().z() 
                                                    ,table_to_bridge.getRotation().w() );
  ROS_DEBUG("bridge_to_base: (%f %f %f)(%f %f %f %f)",base_to_bridge.getOrigin().x() 
                                                    ,base_to_bridge.getOrigin().y() 
                                                    ,base_to_bridge.getOrigin().z() 
                                                    ,base_to_bridge.getRotation().x() 
                                                    ,base_to_bridge.getRotation().y() 
                                                    ,base_to_bridge.getRotation().z() 
                                                    ,base_to_bridge.getRotation().w() );

  // fill in base_pose in table frame (transform from table to bridge)
  geometry_msgs::Pose base_pose;
  base_pose.position.x    = table_to_base_link.getOrigin().x();
  base_pose.position.y    = table_to_base_link.getOrigin().y();
  base_pose.position.z    = table_to_base_link.getOrigin().z();
  base_pose.orientation.x = table_to_base_link.getRotation().x();
  base_pose.orientation.y = table_to_base_link.getRotation().y();
  base_pose.orientation.z = table_to_base_link.getRotation().z();
  base_pose.orientation.w = table_to_base_link.getRotation().w();
  shot_plan.base_pose.header = cue_ball_position.header;
  shot_plan.base_pose.pose = base_pose;

  ROS_DEBUG("cue_pose: (%f,%f,%f) target_pose: (%f,%f,%f) pocket_pose: (%f,%f,%f) base_pose: (%f,%f,%f)"
           ,cue_ball_position.point.x,cue_ball_position.point.y,cue_ball_position.point.z
           ,target_ball_position.point.x,target_ball_position.point.y,target_ball_position.point.z
           ,target_pocket_position.point.x,target_pocket_position.point.y,target_pocket_position.point.z
           ,base_pose.position.x,base_pose.position.y,base_pose.position.z);

  this->base_pose_marker_.header.frame_id = "table";
  this->base_pose_marker_.header.stamp = ros::Time::now();
  this->base_pose_marker_.ns = "planner_base_pose";
  this->base_pose_marker_.id = target_ball_id+20*pocket_id;
  this->base_pose_marker_.type = visualization_msgs::Marker::ARROW;
  this->base_pose_marker_.pose.position.x    = base_pose.position.x;
  this->base_pose_marker_.pose.position.y    = base_pose.position.y;
  this->base_pose_marker_.pose.position.z    = base_pose.position.z;
  this->base_pose_marker_.pose.orientation.x = base_pose.orientation.x;
  this->base_pose_marker_.pose.orientation.y = base_pose.orientation.y;
  this->base_pose_marker_.pose.orientation.z = base_pose.orientation.z;
  this->base_pose_marker_.pose.orientation.w = base_pose.orientation.w;
  this->base_pose_marker_.scale.x = 0.03;
  this->base_pose_marker_.scale.y = 0.03;
  this->base_pose_marker_.scale.z = 0.03;
  this->base_pose_marker_.color.r = 1.0f;
  this->base_pose_marker_.color.b = 1.0f;
  this->base_pose_marker_.color.a = 1.0;
  this->base_pose_marker_.lifetime = ros::Duration();

  // check base collision with table
  // corner of table rectable region c
  // v1
  // v2
  // point p
  // v = p - c
  {
    double radius = billiards_msgs::Constants::ROBOT_RADIUS;
    double length = billiards_msgs::Constants::TABLE_LENGTH;
    double width = billiards_msgs::Constants::TABLE_WIDTH;
    Eigen::Vector3d v1(length+2*radius,0,0);
    Eigen::Vector3d v2(0,width+2*radius,0);
    Eigen::Vector3d c(-radius,-radius,0);
    Eigen::Vector3d p(base_pose.position.x ,base_pose.position.y ,0);
    Eigen::Vector3d v = p - c;
    if (this->inRectangle(c,v1,v2,v))
    {
      ROS_INFO("shot on ball %d into pocket %d cause pr2 base to collide with table",target_ball_id, pocket_id);
      this->bridge_pose_marker_.color.r = 1.0f;
      this->bridge_pose_marker_.color.g = 0.5f;
      this->bridge_pose_marker_.color.b = 0.5f;
      this->marker_pub_.publish(this->bridge_pose_marker_);
      this->base_pose_marker_.color.r = 1.0f;
      this->base_pose_marker_.color.g = 0.1f;
      this->base_pose_marker_.color.b = 0.1f;
      this->marker_pub_.publish(this->base_pose_marker_);
      return -1;
    }
  }


  // not used for now
  shot_plan.velocity = 1000.;

  bool safety_shot = false;

  // check collision with each ball against
  //  trajectory from cue_ball_position to cue_ball_target_position
  // if shot is not possible or obstructed, return -1.0;
  for (std::vector<billiards_msgs::BallState>::iterator ball = this->table_state_.balls.begin();                                       ball != this->table_state_.balls.end(); ball++)
  if (ball->id != cue_ball_id && ball->id != target_ball_id)
  {
    // check collision with each ball
    geometry_msgs::PointStamped ball_position = ball->point;
    geometry_msgs::PointStamped ball_position_transformed;
    tf_listener_.transformPoint("table",ball_position,ball_position_transformed);

    if (this->getIntersection(cue_ball_position.point, cue_ball_target_position, ball_position_transformed.point))
    {
      ROS_INFO("shot on ball %d into pocket %d is obstructed by ball %d from cue ball to target ball",target_ball_id,
               pocket_id, ball->id);
      this->bridge_pose_marker_.color.r = 1.0f;
      this->bridge_pose_marker_.color.g = 0.5f;
      this->bridge_pose_marker_.color.b = 0.5f;
      this->marker_pub_.publish(this->bridge_pose_marker_);
      this->base_pose_marker_.color.r = 1.0f;
      this->base_pose_marker_.color.g = 0.1f;
      this->base_pose_marker_.color.b = 0.1f;
      this->marker_pub_.publish(this->base_pose_marker_);
      return -1;
    }
    if (this->getIntersection(cue_ball_target_position,target_pocket_position.point, ball_position_transformed.point))
    {
      ROS_INFO("shot on ball %d into pocket %d is obstructed by %d from target ball to pocket",target_ball_id,
               pocket_id, ball->id);
     safety_shot = true;  // mark as backup safety shot, given we can still hit the ball, just not pocket it
     //return -1;
    }
  }

  // as a first cut, score is 1 - shot_angle / 90degrees
  //  so straight shot has score of 1 and 90 deg shots have score of 0
  double score = 0;

  // check that the shot angle is < M_PI/2, etc.
  //  the angle between shot vector and target_ball-to-target_pocket angle 
  //   angle = acos( shot_vector dot target_ball_to_target_pocket_vector )
  //    make sure both vectors are normalized.
  double dot_product = ball_to_pocket_direction.dot(shot_direction);
  dot_product = (dot_product > 1.0) ? 1.0 : (dot_product < -1.0) ? -1.0 : dot_product; // limit to [-1,1]
  double shot_angle = acos(dot_product);
  // ROS_INFO("shot_direction: %f %f %f to_pocket: %f %f %f shot_angle: %f"
  //          ,shot_direction.x(),shot_direction.y(),shot_direction.z()
  //          ,ball_to_pocket_direction.x(),ball_to_pocket_direction.y(),ball_to_pocket_direction.z()
  //          ,shot_angle);
  if (shot_angle > M_PI/2.0)
  {
    ROS_INFO("shot on ball %d into pocket %d has angle %f degrees > 90 degrees, not possible.",target_ball_id,
             pocket_id,shot_angle*180.0/M_PI);
    this->bridge_pose_marker_.color.r = 1.0f;
    this->bridge_pose_marker_.color.g = 0.5f;
    this->bridge_pose_marker_.color.b = 0.5f;
    this->marker_pub_.publish(this->bridge_pose_marker_);
    this->base_pose_marker_.color.r = 1.0f;
    this->base_pose_marker_.color.g = 0.1f;
    this->base_pose_marker_.color.b = 0.1f;
    this->marker_pub_.publish(this->base_pose_marker_);
    return -1;
  }
  else
    score = 1.0 - shot_angle / (M_PI/2.0);

  // check for second stage planning, where we don't want to move the base
  // check if second_stage_shot_angle between x-axis of table and x-axis of bridge is within user specified range
  dot_product = shot_direction.dot(Eigen::Vector3d(1,0,0));
  dot_product = (dot_product > 1.0) ? 1.0 : (dot_product < -1.0) ? -1.0 : dot_product; // limit to [-1,1]
  double second_stage_shot_angle = acos(dot_product);
  if (second_stage_shot_angle > this->shot_angle_max_ || second_stage_shot_angle < this->shot_angle_min_)
  {
    ROS_INFO("shot on ball %d into pocket %d has second stage planning shot angle %f out side of range [%f,%f]",target_ball_id,
             pocket_id,second_stage_shot_angle*180.0/M_PI,this->shot_angle_min_*180/M_PI,this->shot_angle_max_*180/M_PI);
    this->bridge_pose_marker_.color.r = 1.0f;
    this->bridge_pose_marker_.color.g = 0.5f;
    this->bridge_pose_marker_.color.b = 0.5f;
    this->marker_pub_.publish(this->bridge_pose_marker_);
    this->base_pose_marker_.color.r = 1.0f;
    this->base_pose_marker_.color.g = 0.1f;
    this->base_pose_marker_.color.b = 0.1f;
    this->marker_pub_.publish(this->base_pose_marker_);
    return -1;
  }

  // check cue ball target position is inside table
  {
    // assuming the rails invade table space by RAIL_DEPTH? CHECK_ME
    double radius = billiards_msgs::Constants::BALL_RADIUS;
    double rail = 0;//billiards_msgs::Constants::RAIL_DEPTH;
    double length = billiards_msgs::Constants::TABLE_LENGTH;
    double width = billiards_msgs::Constants::TABLE_WIDTH;
    Eigen::Vector3d v1(length-2*radius-2*rail,0,0);
    Eigen::Vector3d v2(0,width-2*radius-2*rail,0);
    Eigen::Vector3d c(radius+rail,radius+rail,0);
    Eigen::Vector3d p(cue_ball_target_position.x, cue_ball_target_position.y ,0);
    Eigen::Vector3d v = p - c;
    if (!this->inRectangle(c,v1,v2,v))
    {
      ROS_INFO("shot on ball %d into pocket %d cause robot to aim outside of table",target_ball_id, pocket_id);
      this->bridge_pose_marker_.color.r = 1.0f;
      this->bridge_pose_marker_.color.g = 0.5f;
      this->bridge_pose_marker_.color.b = 0.5f;
      this->marker_pub_.publish(this->bridge_pose_marker_);
      this->base_pose_marker_.color.r = 1.0f;
      this->base_pose_marker_.color.g = 0.1f;
      this->base_pose_marker_.color.b = 0.1f;
      this->marker_pub_.publish(this->base_pose_marker_);
      return -1;
    }
  }

  // further scale score based on possible metrics:
  //   distance from cue ball to target ball and from target ball to pocket
  score = score / (1 + cue_to_ball_dist);
  score = score / (1 + ball_to_pocket_dist);
  
  // @todo:
  // we can further scale score based on possible metrics:
  //   length of the shot
  //   obstruction to bridge
  //   english
  //   pose certainty
  //   etc.


  // if we get this far, robot can hit the ball, just not make it.  score it very low.
  if (safety_shot) score = score*0.0001; // back up safety shot to any unobstructed ball if not other valid shots

  ROS_INFO("shot on ball %d into pocket %d plausible with shot score %f",target_ball_id,pocket_id,score);


  this->bridge_pose_marker_.scale.x = score;
  this->bridge_pose_marker_.scale.y = score;
  this->bridge_pose_marker_.scale.z = score;
  this->bridge_pose_marker_.color.r = (float)score;
  this->bridge_pose_marker_.color.g = 1.0f;
  this->bridge_pose_marker_.color.b = (float)score;
  this->bridge_pose_marker_.ns = "planner_bridge_pose_good";
  this->marker_pub_.publish(this->bridge_pose_marker_);
  this->base_pose_marker_.scale.x = score;
  this->base_pose_marker_.scale.y = score;
  this->base_pose_marker_.scale.z = score;
  this->base_pose_marker_.color.r = (float)score;
  this->base_pose_marker_.color.g = 1.0f;
  this->base_pose_marker_.color.b = (float)score;
  this->base_pose_marker_.ns = "planner_base_pose_good";
  this->marker_pub_.publish(this->base_pose_marker_);
  return score;
}

bool SimpleShotPlanner::getIntersection(geometry_msgs::Point start_point, geometry_msgs::Point end_point, geometry_msgs::Point sphere_center)
{
  // checks intersection from shot_vector to the sphere, return true if there exists intersection
  // this is the same as checking if sphere center is inside a rectangle defined by making the trajectory
  // one diameter wide on each side.
  // define rectable by one vertex c and two vectors v1,v2 from the vertex.
  // call the sphere center p, and v = p - c
  // then, p is in the rectangle if and only if
  //   0<=dot_product(v,v1)<=dot_product(v1,v1) and 0<=dot_product(v,v2)<=dot_product(v2,v2)
  Eigen::Vector3d v1(end_point.x - start_point.x
                    ,end_point.y - start_point.y
                    ,end_point.z - start_point.z);
  Eigen::Vector3d v2 = Eigen::Vector3d(0,0,1).cross(v1);
  v2.normalize(); v2 *= 4.0*this->ball_radius_;  // pathway is 4 radii wide
  //ROS_INFO("debug: v1(%f,%f,%f) v2(%f,%f,%f)",v1.x(),v1.y(),v1.z(),v2.x(),v2.y(),v2.z());

  Eigen::Vector3d c(start_point.x ,start_point.y ,start_point.z);
  c -= 0.5*v2;

  Eigen::Vector3d p(sphere_center.x ,sphere_center.y ,sphere_center.z);
  Eigen::Vector3d v = p - c;
  return this->inRectangle(c,v1,v2,v);
}
bool SimpleShotPlanner::inRectangle(Eigen::Vector3d c,Eigen::Vector3d v1,Eigen::Vector3d v2,Eigen::Vector3d v)
{
  // checks intersection from shot_vector to the sphere, return true if there exists intersection
  // intersection_point = find point of intersection of vector and sphere;
  return (v2.dot(v2) >= v.dot(v2) && v.dot(v2) >= 0 && v1.dot(v1) >= v.dot(v1) && v.dot(v1) >= 0);
}

