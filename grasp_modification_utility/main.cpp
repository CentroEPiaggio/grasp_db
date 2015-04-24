#include <iostream>
#include "ros/ros.h"
#include <string>
#include "grasp_modification_utility.h"
#include "dual_manipulation_shared/serialization_utils.h"
#include "tf_conversions/tf_kdl.h"

int main(int argc, char** argv)
{
    sleep(1);

    if( !ros::isInitialized() )
    {
	ros::init( argc, argv, "grasp_modification_utility", ros::init_options::AnonymousName );
    }
    
    ros::AsyncSpinner spin(1);
    spin.start();
    GMU gmu;

    ROS_INFO_STREAM("This is a utility to modify one serialized grasp (post grasp pose not included)");

    dual_manipulation_shared::grasp_trajectory grasp_msg;
    int obj_id;
    int grasp_id;
    std::string file_name;

    while(1)
    {
        gmu.clear();

	while(1)
	{
	    ROS_INFO_STREAM("Select object id:");
	    std::cin>>obj_id;
	    
	    ROS_INFO_STREAM("Select grasp id:");
	    std::cin>>grasp_id;

	    file_name = "object" + std::to_string(obj_id) + "/grasp" + std::to_string(grasp_id);
	    
	    if(deserialize_ik(grasp_msg,file_name))
	    {
		ROS_INFO_STREAM("Deserialization object" + std::to_string(obj_id) + "/grasp" + std::to_string(grasp_id) << " OK! ... You can modify the grasp in Rviz");
		
		if(!gmu.db_mapper.Objects.count(obj_id))
		{
		    ROS_WARN_STREAM("Object "<<grasp_msg.object_db_id<<" is not in the database! . . . Retry!");
		    continue;
		}
		break;
	    }
	    else
	    {
		ROS_WARN_STREAM("Error in deserialization object" + std::to_string(obj_id) + "/grasp" + std::to_string(grasp_id) << "! . . . Retry!");
		continue;
	    }
	}
	
	gmu.set_object(obj_id);
	gmu.set_hands(grasp_msg.ee_pose);

	gmu.publish_object();
	gmu.publish_hands();

	ROS_INFO_STREAM("Enter any key to not modify the grasp");
	ROS_INFO_STREAM("Type \"exit\" to quit the program");
	ROS_INFO_STREAM("Type \"save\" otherwise");
	std::string cmd;
	std::cin>>cmd;

	if(cmd=="save")
	{
	    geometry_msgs::Pose object;
	    gmu.get_object(object);
	    gmu.get_hands(grasp_msg.ee_pose);

	    KDL::Frame world_object, world_hand;
	    tf::poseMsgToKDL(object,world_object);
	    
	    for(int j=0; j<grasp_msg.ee_pose.size(); j++) //if we have changed the object position we want to update all the hand position w.r.t. the object
	    {
		tf::poseMsgToKDL(grasp_msg.ee_pose.at(j),world_hand);
		tf::poseKDLToMsg(world_object.Inverse()*world_hand,grasp_msg.ee_pose.at(j));
	    }

	    if(!serialize_ik(grasp_msg,file_name))
	    {
		ROS_WARN_STREAM("Error in serialization object" + std::to_string(obj_id) + "/grasp" + std::to_string(grasp_id) << "! . . . Aborting!");
	    }
	    else
	    {
		ROS_INFO_STREAM("Serialization object" + std::to_string(obj_id) << "/grasp" << grasp_id << " OK!");
	    }
	}
	if(cmd=="exit")
	{
	    break;
	}

	ROS_INFO_STREAM("Enter any key to modify another grasp");
	ROS_INFO_STREAM("Type \"exit\" to quit the program");

	std::cin>>cmd;
	
	if(cmd=="exit")
	{
	    break;
	}
    }
    
    ROS_INFO_STREAM("Bye! Thanks for using this terrific program.");
}