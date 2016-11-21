/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Alessandro Settimi, Hamal Marino, Mirko Ferrati, Centro di Ricerca "E. Piaggio", University of Pisa
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "grasp_modification_utility.h"
#include "tf_conversions/tf_kdl.h"
#include "tf/tf.h"
#include <dual_manipulation_shared/parsing_utils.h>

#define FINAL_OFFSET_X 1.0

GMU::GMU():server("grasp_modification_utility_interactive_marker")
{
    objects_active.store(true);

    obj_sub = node.subscribe("grasp_modification_utility_object",2,&GMU::update_position,this);
    hand_sub = node.subscribe("grasp_modification_utility_hands",10,&GMU::update_position,this);
    object_marker_pub = node.advertise<visualization_msgs::Marker>( "grasp_modification_utility_object", 0 );
    hands_marker_pub = node.advertise<visualization_msgs::Marker>( "grasp_modification_utility_hands", 0 );
    im_sub_obj = node.subscribe("grasp_modification_utility_interactive_marker/feedback",1,&GMU::im_callback,this);
    js_sub = node.subscribe("joint_states", 1, &GMU::publishTF, this);
    fake_im_pub = node.advertise<visualization_msgs::InteractiveMarkerFeedback>("grasp_modification_utility_interactive_marker/feedback",0);

    transform_.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform_.setRotation( tf::Quaternion( 0.0, 0.0, 0.0, 1.0) );
    
    node.getParam("hand_mesh_path", hand_mesh_path_);
    node.param("hand_mesh_scale", hand_mesh_scale_, 1.0);
    node.getParam("ee_link_name", ee_link_name_);

    XmlRpc::XmlRpcValue params;
    std::string database_name;
    if( node.getParam("dual_manipulation_parameters", params) )
      parseSingleParameter(params, database_name, "database_name");

    //std::string db_name("full");
    db_writer = boost::shared_ptr<databaseWriter>( new databaseWriter( database_name ) );
    db_mapper = boost::shared_ptr<databaseMapper>( new databaseMapper( database_name ) );
}

void GMU::set_object(int id)
{
    obj_id = id;
}

void GMU::set_hands(std::vector< geometry_msgs::Pose > hands, geometry_msgs::Pose final_hand)
{
    for(auto hand:hands) hand_poses.push_back(hand);
    hand_final_pose = final_hand;
    hand_final_pose.position.x += FINAL_OFFSET_X;
    number_of_waypoints = hand_poses.size()+1;
}

void GMU::get_object(geometry_msgs::Pose& obj, geometry_msgs::Pose& final_obj)
{
    obj = obj_pose;
    final_obj = obj_final_pose;
    final_obj.position.x -= FINAL_OFFSET_X;
}

geometry_msgs::Pose GMU::get_wp(int i)
{
    if(i<hand_poses.size())
	return hand_poses.at(i);
    else
    {
      ROS_ERROR_STREAM("Error in getting wp @ "<<__FILE__<<": "<<__LINE__);
      return geometry_msgs::Pose();
    }
}

void GMU::set_wp(int i, geometry_msgs::Pose wp)
{
    if(i<hand_poses.size())
	hand_poses.at(i)=wp;
    else
    {
      ROS_ERROR_STREAM("Error in setting wp @ "<<__FILE__<<": "<<__LINE__);
      return;
    }
}

void GMU::delete_wp(int i)
{
    std::vector<geometry_msgs::Pose> new_hand_poses;

    for(int j=0;j<hand_poses.size();j++)
	if(j!=i) new_hand_poses.push_back(hand_poses.at(j));

    hand_poses=new_hand_poses;
}

void GMU::add_extra_wp(int i)
{
    std::vector<geometry_msgs::Pose> new_hand_poses;

    for(int j=0;j<hand_poses.size();j++)
    {
        new_hand_poses.push_back(hand_poses.at(j));
	if(j==i) new_hand_poses.push_back(hand_poses.at(j));
    }

    hand_poses=new_hand_poses;
}

void GMU::get_hands(std::vector< geometry_msgs::Pose >& hands, geometry_msgs::Pose& final_hand)
{
    hands.clear();
    for(int i=0;i<hand_poses.size();i++) hands.push_back(hand_poses.at(i));
    final_hand = hand_final_pose;
    final_hand.position.x -= FINAL_OFFSET_X;
}

void GMU::setCurrentWaypoint(int wp)
{
    current_waypoint=wp;
}

void GMU::publish_object()
{
    std::string path = db_mapper->Objects.at(obj_id).mesh_path;

    visualization_msgs::Marker& marker(object_marker);
    marker.header.frame_id="world";
    marker.lifetime=ros::DURATION_MAX;
    marker.type=visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource=path.c_str();
    marker.scale.x=1;
    marker.scale.y=1;
    marker.scale.z=1;
    marker.id=0;
    marker.ns="object";
    marker.pose = geometry_msgs::Pose();
    marker.pose.orientation.w=1;
    marker.color.a=1;
    marker.color.r=0;
    marker.color.g=1;
    marker.color.b=0;
    object_marker_pub.publish(marker);
    
    // final object configuration
    marker.id = 1;
    marker.ns = "final_object";
    marker.pose.position.x += FINAL_OFFSET_X;
    marker.color.g=0;
    marker.color.b=1;
    object_marker_pub.publish(marker);
}

void GMU::force_im_update()
{
    for(int i=0;i<hand_poses.size();i++)
    {
    	if(i==current_waypoint)
	{
	    visualization_msgs::InteractiveMarkerFeedback fake_fb;
	    fake_fb.pose = hand_poses.at(i);
	    fake_fb.marker_name = "hands";
	    fake_im_pub.publish(fake_fb);
	}
    }
}

void GMU::publish_hands()
{
    int i=0;
    visualization_msgs::Marker marker;
    marker.header.frame_id="world";
    marker.lifetime=ros::DURATION_MAX;

    marker.action=3; //clearing the display
    hands_marker_pub.publish(marker);

    marker.action=visualization_msgs::Marker::ADD;
    marker.type=visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource=hand_mesh_path_.c_str();
    marker.scale.x=hand_mesh_scale_;
    marker.scale.y=hand_mesh_scale_;
    marker.scale.z=hand_mesh_scale_;
    
    for(auto hand_pose:hand_poses)
    {
	marker.id=i;
	marker.ns="hands";

	marker.pose = hand_pose;

	if(hand_poses.size()!=1)
	{
	    marker.color.a=1;
	    marker.color.r=0+(double)i/((double)hand_poses.size()-1);
	    marker.color.g=0;
	    marker.color.b=1-(double)i/((double)hand_poses.size()-1);
	}
	else
	{
	    marker.color.a=1;
	    marker.color.r=1;
	    marker.color.g=0;
	    marker.color.b=0;
	}

	hands_marker_pub.publish(marker);

	i++;
    }
    
    // final hand configuration
    marker.id = i;
    marker.ns = "final_hand";
    marker.pose = hand_final_pose;
    marker.color.a=1;
    marker.color.r=1;
    marker.color.g=1;
    marker.color.b=0;
    hands_marker_pub.publish(marker);
}

void GMU::toggle_objects_interaction(bool on)
{
    objects_active.store(on);
}

void GMU::update_position(const visualization_msgs::Marker &marker_)
{
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/world";

    if( (marker_.ns=="object" || marker_.ns=="final_object")  ||
	(marker_.ns=="hands" && current_waypoint==marker_.id) ||
	(marker_.ns=="final_hand") )
      int_marker.name = marker_.ns;
    else
      return;

    int_marker.description = "";
    int_marker.scale=0.2;

    int_marker.controls.clear();
    
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back( marker_);

    int_marker.controls.push_back( box_control );

    if((int_marker.name=="object" || int_marker.name=="final_object") && !objects_active.load())
    {
        server.insert(int_marker);
	server.applyChanges();
	return;
    }

    visualization_msgs::InteractiveMarkerControl control;
    
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    
    int_marker.pose=marker_.pose;

    server.insert(int_marker);

    server.applyChanges();
}


void GMU::im_callback(const visualization_msgs::InteractiveMarkerFeedback& feedback)
{   
    if(feedback.marker_name=="object")
    {
	obj_pose = feedback.pose;
	
	// change orientation of the post-grasped object accordingly to the initial object...
	obj_final_pose.orientation = obj_pose.orientation;
	object_marker.pose = obj_final_pose;
	object_marker_pub.publish(object_marker);
    }
    else if(feedback.marker_name=="final_object")
    {
	obj_final_pose = feedback.pose;
    }
    else
    {
      static tf::TransformListener tf;
      tf::StampedTransform hand_palm;
      double timeout = 5.0;
      if(!tf.waitForTransform(ee_link_name_,"hand",ros::Time(0), ros::Duration(timeout)))
      {
          ROS_WARN_STREAM(__func__ << " : failed to wait for transform between \'" << ee_link_name_ << "\' and \'hand\' > setting to Idendity");
          hand_palm.setIdentity();
      }
      else
      {
          tf.lookupTransform(ee_link_name_,"hand", ros::Time(0), hand_palm);
      }
      transform_.setOrigin( tf::Vector3(feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z) );
      transform_.setRotation( tf::Quaternion( feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w) );
      transform_.mult(transform_,hand_palm);
      if(feedback.marker_name=="final_hand")
      {
	hand_final_pose = feedback.pose;
	publish_hands();
      }
      else
      {
	hand_poses.at(current_waypoint)=feedback.pose;
	publish_hands();
      }
    }
}

void GMU::publishTF(const sensor_msgs::JointState &msg)
{
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "world", "hand"));
}

void GMU::clear()
{
    obj_pose = geometry_msgs::Pose();
    obj_pose.orientation.w=1;
    obj_final_pose = obj_pose;
    obj_final_pose.position.x += FINAL_OFFSET_X;
    hand_final_pose = obj_pose;
    hand_poses.clear();
}

GMU::~GMU()
{

}
