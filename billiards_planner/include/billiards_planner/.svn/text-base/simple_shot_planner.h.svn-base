#include<stdio.h>
#include<stdlib.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/package.h>
#include <roslib/Clock.h>

#include<roslib/Header.h>
#include<billiards_msgs/BallState.h>
#include<billiards_msgs/TableState.h>
#include<billiards_msgs/ShotPlan.h>
#include<billiards_msgs/Constants.h>
#include<billiards_planner/PlanOneShot.h>


#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>

#include <tf/transform_listener.h>

#include <billiards_msgs/PlanShotAction.h>
#include <actionlib/server/simple_action_server.h>

#include <visualization_msgs/Marker.h>

typedef actionlib::SimpleActionServer<billiards_msgs::PlanShotAction> ActionServer;

namespace simple_shot_planner
{
  class SimpleShotPlanner
  {
    public:
      SimpleShotPlanner();
      ~SimpleShotPlanner();

      /// \brief plan next shot
      bool planNextShot(billiards_planner::PlanOneShot::Request &req,billiards_planner::PlanOneShot::Response &res);

      /// \brief plan one shot
      double planOneShot(const geometry_msgs::PointStamped cue_ball_position
                        ,const geometry_msgs::PointStamped target_ball_position
                        ,const geometry_msgs::PointStamped target_pocket_position
                        ,int cue_ball_id, int target_ball_id, int pocket_id
                        ,billiards_msgs::ShotPlan& shot_plan);

      bool getIntersection(geometry_msgs::Point start_point, geometry_msgs::Point end_point, geometry_msgs::Point sphere_center);
      bool inRectangle(Eigen::Vector3d c,Eigen::Vector3d v1,Eigen::Vector3d v2,Eigen::Vector3d v);

    private:
      /// \brief plan shot action
      void actionGoalCallback();

      /// \brief plan shot action
      void actionPreemptCallback();

      /// \brief ros queue thread for this node
      void queueThread();

      /// \brief advertise services
      void AdvertiseServices();

      /// \brief subscribe to this topic
      void SubscribeTopics();

      /// \brief update per topic
      void updateTableState(const billiards_msgs::TableState::ConstPtr& table_state);
      ros::NodeHandle    rosnode_;
      tf::TransformListener tf_listener_;
      ros::CallbackQueue queue_;
      boost::thread*     callback_queue_thread_;
      ros::ServiceServer plan_next_shot_service_;
      ros::Subscriber    table_state_listener_;
      /// \brief A mutex to lock access to fields that are used in ROS message callbacks
      boost::mutex lock_;
      bool table_state_initialized_;
      billiards_msgs::TableState table_state_;
      double shot_angle_min_;
      double shot_angle_max_;
      double ball_radius_;
      double bridge_distance_from_cue_ball_;
      std::vector<billiards_msgs::PocketState> pocket_states_; // in table frame

      // as an action server
      billiards_msgs::PlanShotGoalConstPtr action_goal_;
      billiards_msgs::PlanShotResult action_result_;
      billiards_msgs::PlanShotFeedback action_feedback_;
      ActionServer action_server_;
      bool found_valid_shot_plan;
      double best_score;
      billiards_msgs::ShotPlan best_shot_plan;

      visualization_msgs::Marker bridge_pose_marker_;
      visualization_msgs::Marker base_pose_marker_;

      ros::Publisher marker_pub_;
  };
}

