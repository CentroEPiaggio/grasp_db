#include "grasp_modification_utility.h"

GMU::GMU():server("grasp_modification_utility_interactive_marker")
{
    obj_sub = node.subscribe("/grasp_modification_utility_object",1,&GMU::update_position,this);
    hand_sub = node.subscribe("/grasp_modification_utility_hands",10,&GMU::update_position,this);
    object_marker_pub = node.advertise<visualization_msgs::Marker>( "/grasp_modification_utility_object", 0 );
    hands_marker_pub = node.advertise<visualization_msgs::Marker>( "/grasp_modification_utility_hands", 0 );
    im_sub_obj = node.subscribe("/grasp_modification_utility_interactive_marker/feedback",1,&GMU::im_callback,this);
    js_sub = node.subscribe("/joint_states", 1, &GMU::publishTF, this);

    transform_.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform_.setRotation( tf::Quaternion( 0.0, 0.0, 0.0, 1.0) );

}

void GMU::set_object(int id)
{
    obj_id = id;
}

void GMU::set_hands(std::vector< geometry_msgs::Pose > hands)
{
    for(auto hand:hands) hand_poses.push_back(hand);
}

void GMU::get_object(geometry_msgs::Pose& obj)
{
    obj = obj_pose;
}

void GMU::get_hands(std::vector< geometry_msgs::Pose >& hands)
{
    for(int i=0;i<hand_poses.size();i++) hands.at(i)=hand_poses.at(i);
}

void GMU::publish_object()
{
    std::string path = "package://asus_scanner_models/";
    path.append(std::get<1>(db_mapper.Objects.at(obj_id)));

    visualization_msgs::Marker marker;
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
}

void GMU::publish_hands()
{
    std::string path = "package://soft_hand_description/meshes/palm_right.stl";
    int i=0;
    for(auto hand_pose:hand_poses)
    {
        visualization_msgs::Marker marker;
	marker.header.frame_id="world";
	marker.lifetime=ros::DURATION_MAX;

	marker.type=visualization_msgs::Marker::MESH_RESOURCE;
	marker.mesh_resource=path.c_str();
	marker.scale.x=0.001;
	marker.scale.y=0.001;
	marker.scale.z=0.001;

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
}

void GMU::update_position(const visualization_msgs::Marker &marker_)
{
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/world";
    int_marker.name = (marker_.ns=="hands")?std::to_string(marker_.id):marker_.ns;
    int_marker.description = "";
    int_marker.scale=0.2;

    int_marker.controls.clear();

    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back( marker_);

    int_marker.controls.push_back( box_control );

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
    }
    else
    {
	hand_poses.at(std::stoi(feedback.marker_name))=feedback.pose;
    transform_.setOrigin( tf::Vector3(feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z) );
    transform_.setRotation( tf::Quaternion( feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w) );
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

    hand_poses.clear();
}

GMU::~GMU()
{

}
