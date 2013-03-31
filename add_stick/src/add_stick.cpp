
#include <ros/ros.h>
#include <stdio.h>
#include <vector>

#include "planning_environment/models/collision_models.h"

//#include <mapping_msgs/CollisionObject.h>
#include <geometric_shapes/shapes.h>
//#include <tinyxml/tinyxml.h>
#include <urdf/model.h>
#include <planning_models/kinematic_model.h>
#include <tf/tf.h>
#include <planning_environment/util/construct_object.h>
#include <gazebo/GetModelState.h>
#include <gazebo/GetLinkState.h>
#include <gazebo_msgs/GetLinkState.h>
//#include <urdf/link.h>

shapes::Shape* constructShape(const urdf::Geometry *geom)
{
  ROS_ASSERT(geom);
 
  shapes::Shape *result = NULL;
  if(geom->type == urdf::Geometry::BOX)
  {
        ROS_INFO("BOX");
    urdf::Vector3 dim = dynamic_cast<const urdf::Box*>(geom)->dim;
    result = new shapes::Box(dim.x, dim.y, dim.z);
  }
  else if(geom->type == urdf::Geometry::SPHERE)
  {
        ROS_INFO("SPHERE");
    result = new shapes::Sphere(dynamic_cast<const urdf::Sphere*>(geom)->radius);
  }
  else if(geom->type == urdf::Geometry::CYLINDER)
  {
        ROS_INFO("CYLINDER");
    result = new shapes::Cylinder(dynamic_cast<const urdf::Cylinder*>(geom)->radius, dynamic_cast<const urdf::Cylinder*>(geom)->length);
  }
  else if(geom->type == urdf::Geometry::MESH)
  {
        //you can find the code in motion_planning_common/planning_models/kinematic_models.cpp
  ROS_INFO("1\n");	
        ROS_INFO("MESH --- currently not supported");
  }
  else
  {
    ROS_ERROR("Unknown geometry type: %d", (int)geom->type);
  }
    
  return result;
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "addURDF");
  const std::string frame_id = "/base_footprint";
  //const std::string frame_id = "/map";

  ros::NodeHandle nh;

  ros::Publisher object_in_map_pub_;
  object_in_map_pub_  = nh.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 20);
  

  std::string parameter_name = "stick_urdf";//../../pr2_billiards_sim/urdf/stick.urdf";
  std::string model_name = "stick";
  
  urdf::Model model;
  
  model.initParam(parameter_name);
 
  ROS_INFO("Successfully parsed urdf file");
  
  std::vector< boost::shared_ptr< urdf::Link > > URDF_links;
  model.getLinks(URDF_links);
  std::map< std::string, boost::shared_ptr< urdf::Joint > > URDF_joints = model.joints_;
  std::map< std::string, boost::shared_ptr< urdf::Joint > >::iterator joints_it;


  //tranfo between links is in joints->origin!
  //access to Joint-Information
  for(joints_it=URDF_joints.begin() ; joints_it != URDF_joints.end(); joints_it++)
  {
    ROS_INFO("Joint name: %s", (*joints_it).first.c_str());
    ROS_INFO("\t origin: %f,%f,%f", (*joints_it).second->parent_to_joint_origin_transform.position.x, (*joints_it).second->parent_to_joint_origin_transform.position.y, (*joints_it).second->parent_to_joint_origin_transform.position.z);
  }

  //ros::service::waitForService("/gazebo/get_model_state");
  ros::service::waitForService("gazebo/get_link_state");
  //ros::service::waitForService("/cob3_environment_server/get_state_validity");  //just to make sure that the environment_server is there!
  ROS_INFO("service ready!!");
  //access to tranformation /world to /root_link (table_top)
  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetLinkState>("gazebo/get_link_state");
  gazebo_msgs::GetLinkState srv;

 // srv.request.model_name = model_name;
  
  arm_navigation_msgs::CollisionObject collision_object;
  collision_object.id = model_name + "_object";
  collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  //collision_object.operation.operation = mapping_msgs::CollisionObjectOperation::REMOVE;
  collision_object.header.frame_id = frame_id;
  collision_object.header.stamp = ros::Time::now();
  collision_object.shapes.resize(URDF_links.size());
  collision_object.poses.resize(URDF_links.size());
  
  ROS_INFO("1 URDF_links %d\n",URDF_links.size());	
  joints_it=URDF_joints.begin();
while(ros::ok())
{
  for(unsigned int i=0; i<URDF_links.size(); i++)
  {
          urdf::Link current_link = *URDF_links[i];
          ROS_INFO("Current Link: %s", current_link.name.c_str());
/*
          if(current_link.name == "lever_stick")
          {
                  ROS_INFO("Dealing with FU lever stick dummy_link...");
                  continue;
          }
*/
  //        boost::shared_ptr< urdf::Joint > current_parent_joint = current_link.parent_joint;
          //ROS_INFO("Current Parent Joint: %s", current_parent_joint->name.c_str());

          
          //fill CollisionObject for each link
          shapes::Shape *current_shape;
          current_shape = constructShape(current_link.collision->geometry.get());
          ROS_INFO("shape.type: %d", current_shape->type);
          
//**************************************************************
// Making a service call for getting link state of current link with respect to frame_id
          tf::Pose pose1;
  srv.request.link_name = current_link.name;//;.c_str();
  srv.request.reference_frame = frame_id;
//**********************************************************
  if (client.call(srv))
  {
//        ROS_INFO("URDFPose (x,y,z): (%f,%f,%f)", srv.response.pose.position.x, srv.response.pose.position.y, srv.response.pose.position.z);
        collision_object.poses[i] = srv.response.link_state.pose;//msg_pose_stamped.pose;
/*	pose1.position.x = srv.response.link_state.pose.position.x;
*/	
  }
  else
  {
        ROS_ERROR("Failed to call service get_link_state");
   //     ros::shutdown();
  }
          //tf::Transform world2dummy;
          //tf::Transform dummy2link;
          
	  //tf::Pose pose2;
         // pose2.mult(world2dummy, dummy2link);
/*
          tf::Stamped<tf::Pose> stamped_pose_in;
          stamped_pose_in.stamp_ = ros::Time::now();
          stamped_pose_in.frame_id_ = frame_id;
          stamped_pose_in.setData(pose1);
  */        
	ROS_INFO("11");
	  arm_navigation_msgs::Shape msg_shape;	
//          geometric_shapes::shapes msg_shape;
	ROS_INFO("22");
          planning_environment::constructObjectMsg(current_shape, msg_shape);
          
	ROS_INFO("33");
       //   geometry_msgs::PoseStamped msg_pose_stamped;
          //tf::poseStampedTFToMsg (transformed_pose, msg_pose_stamped);
     //     tf::poseStampedTFToMsg (stamped_pose_in, msg_pose_stamped);
          
          collision_object.shapes[i] = msg_shape;
        
	ROS_INFO("44");
          //object_in_map_pub_.publish(collision_object);
        
          ROS_INFO("Should have published");
  }
	
  //ROS_INFO("3\n");
	  ros::Rate loop_rate(20);
          object_in_map_pub_.publish(collision_object);
//	  ros::spinOnce();
	  loop_rate.sleep();
}	
  //ros::spin();		
//  ros::shutdown();
}
