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

	ROS_INFO_STREAM("This is a utility to modify one serialized grasp (post grasp pose is with the blue object - grasp trajectory with the green one)");

	dual_manipulation_shared::grasp_trajectory grasp_msg;
	int obj_id;
	int grasp_id;
    int new_grasp_id;
    int ee_id;
	std::string file_name;
	std::string mode;
	std::string new_grasp_name;

	while(1)
	{
		gmu.clear();
		ROS_INFO_STREAM("Do you want to [e] edit or [c] copy a grasp?");
		std::cin >> mode;

		while(1)
		{

            if( mode == "e" )
			{
				ROS_INFO_STREAM("Select object id:");
				std::cin >> obj_id;
				
				ROS_INFO_STREAM("Select grasp id:");
				std::cin >> grasp_id;
			}
            if( mode == "c" )
			{
				ROS_INFO_STREAM("Select object id:");
				std::cin >> obj_id;
				
                ROS_INFO_STREAM("Select grasp id you want to copy from (new added grasps are not considered!):");
				std::cin >> grasp_id;

                // ASSUMING WE ARE WORKING WITH THE FULL DATABASE; AGREE ON GRASP ID'ING!
                grasp_id = grasp_id;

                // ROS_INFO_STREAM("How do you want to call the new grasp?");
                // std::cin >> new_grasp_name;
			}

			file_name = "object" + std::to_string( obj_id ) + "/grasp" + std::to_string( grasp_id );
			if( deserialize_ik( grasp_msg, file_name ) )
			{
				ROS_INFO_STREAM("Deserialization object" + std::to_string( obj_id ) + "/grasp" + std::to_string( grasp_id ) << " OK! ... You can modify the grasp in Rviz");
				// std::cout << "grasp_msg" << grasp_msg << std::endl;
				if( !gmu.db_mapper->Objects.count( obj_id ) )
				{
					ROS_WARN_STREAM("Object " << grasp_msg.object_db_id << " is not in the database! . . . Retry!");
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
		KDL::Frame obj_hand_postGrasp;
		tf::poseMsgToKDL( grasp_msg.attObject.object.mesh_poses.front(), obj_hand_postGrasp );
		obj_hand_postGrasp = obj_hand_postGrasp.Inverse();
		geometry_msgs::Pose postGrasp_pose;
		tf::poseKDLToMsg( obj_hand_postGrasp, postGrasp_pose );
		gmu.set_hands( grasp_msg.ee_pose, postGrasp_pose );

		gmu.publish_object();
		gmu.publish_hands();

		ROS_INFO_STREAM("Enter any key to not modify the grasp");
		ROS_INFO_STREAM("Type \"exit\" to quit the program");
		ROS_INFO_STREAM("Type \"save\" otherwise");
		std::string cmd;
		std::cin >> cmd;

		if( cmd=="save" )
		{
            if( mode == "c")
            {
                std::cout << "grasp_id: " << grasp_id << std::endl;
                std::cout << "copying name: " << std::get<2>( gmu.db_mapper->Grasps.at(grasp_id) ) << std::endl;
                new_grasp_name = std::get<2>( gmu.db_mapper->Grasps.at( grasp_id ) ) + " (copy)";
                std::cout << "new name is: " << new_grasp_name << std::endl;
                ee_id = std::get<1>( gmu.db_mapper->Grasps.at( grasp_id ) );
                std::cout << "copying ee_id: " << ee_id << std::endl;
                std::cout << "writing to database..." << std::endl;
                new_grasp_id = gmu.db_writer->writeNewGrasp( obj_id, ee_id, new_grasp_name);
                if(!(new_grasp_id > 0) )
                    ROS_ERROR_STREAM("Couldn't write the database");
                file_name = "object" + std::to_string( obj_id ) + "/grasp" + std::to_string( new_grasp_id );
            }

			geometry_msgs::Pose object,final_object, final_hand;
			gmu.get_object(object, final_object);
			gmu.get_hands(grasp_msg.ee_pose, final_hand);

			KDL::Frame world_object, world_hand;
			tf::poseMsgToKDL(object, world_object);
			
            for( int j=0; j < grasp_msg.ee_pose.size(); j++ ) //if we have changed the object position we want to update all the hand position w.r.t. the object
			{
				tf::poseMsgToKDL(grasp_msg.ee_pose.at(j),world_hand);
				tf::poseKDLToMsg(world_object.Inverse()*world_hand,grasp_msg.ee_pose.at(j));
			}
			
			// also modify post-grasp pose
			tf::poseMsgToKDL(final_object,world_object);
			tf::poseMsgToKDL(final_hand,world_hand);
			tf::poseKDLToMsg(world_hand.Inverse()*world_object,grasp_msg.attObject.object.mesh_poses.front());

            if( !serialize_ik( grasp_msg, file_name ) )
			{
                if( mode == "c")
                {
                    ROS_WARN_STREAM("Error in serialization object" + std::to_string(obj_id) + "/grasp" + std::to_string(new_grasp_id) << "! . . . Aborting!");
                }
                else
                {
                    ROS_WARN_STREAM("Error in serialization object" + std::to_string(obj_id) + "/grasp" + std::to_string(grasp_id) << "! . . . Aborting!");
                }
			}
			else
			{
                if( mode == "c")
                {
                    ROS_INFO_STREAM("Serialization object" + std::to_string(obj_id) << "/grasp" << new_grasp_id << " OK!");
                }
                else
                {
                    ROS_INFO_STREAM("Serialization object" + std::to_string(obj_id) << "/grasp" << grasp_id << " OK!");
                }
			}
		}
        if( cmd == "exit" )
		{
			break;
		}

		ROS_INFO_STREAM("Enter any key to modify another grasp");
		ROS_INFO_STREAM("Type \"exit\" to quit the program");

        std::cin >> cmd;
		
        if( cmd == "exit" )
		{
			break;
		}
	}

	ROS_INFO_STREAM("Bye! Thanks for using this terrific program.");
	return 0;
}
