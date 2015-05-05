#include <sqlite3.h>

#include "grasps_serializer.h"
#include "dual_manipulation_shared/grasp_trajectory.h"
#include "dual_manipulation_shared/serialization_utils.h"

#define ABSURD_TIME 100.0
#define THREAD_TIME 10000

// public

bool GraspsSerializer::parseGraspInfo(XmlRpc::XmlRpcValue grasps)
{
  if(grasp_info_set_)
  {
    ROS_WARN_STREAM("GraspsSerializer::parseGraspInfo : grasp_info already set!");
    return false;
  }

  // parse dataset package name
  ROS_ASSERT(grasps["dataset"].getType() == XmlRpc::XmlRpcValue::TypeString);
  dataset_ = std::string( grasps["dataset"] );

  // parse object id and name
  ROS_ASSERT(grasps["object_name"].getType() == XmlRpc::XmlRpcValue::TypeString);
  object_name_ = std::string( grasps["object_name"] );

  // open database (this is not actually parsing, but it is needed for validation, but it requires the object name parse)
  // std::string path = ros::package::getPath("dual_manipulation_grasp_db");
  std::string db_name(object_name_ + ".db");
  openDB( db_name );

  ROS_ASSERT(grasps["object_id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
  object_id_ = grasps["object_id"];

  // check object id in DB
  if(db_mapper_->Objects.count(object_id_) == 0)
  {
    ROS_ERROR_STREAM("GraspsSerializer::parseGraspInfo : No object present in the DB with ID " << object_id_);
    return false;
  }

  // parse tf info
  ROS_ASSERT(grasps["reference_frame"].getType() == XmlRpc::XmlRpcValue::TypeString);
  world_tf_ = std::string( grasps["reference_frame"] );

  ROS_ASSERT(grasps["end_effector_frame"].getType() == XmlRpc::XmlRpcValue::TypeString);
  hand_tf_ = std::string( grasps["end_effector_frame"] );

  ROS_ASSERT(grasps["object_frame"].getType() == XmlRpc::XmlRpcValue::TypeString);
  object_tf_ = std::string( grasps["object_frame"] );  
  
  // parse grasps into the struct
  ROS_ASSERT(grasps["grasps"].getType() == XmlRpc::XmlRpcValue::TypeArray);
  int n_grasps = grasps["grasps"].size();

  // rec_grasps_.resize( n_grasps );

  for( int i = 0; i < n_grasps; ++i )
  {
    XmlRpc::XmlRpcValue current_grasp;
    current_grasp = grasps["grasps"][i];

    rec_grasp my_grasp;

    // parse end effector id
    ROS_ASSERT(current_grasp["end_effector_id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    my_grasp.end_effector_id =  current_grasp["end_effector_id"];

    // check end effector id in DB
    if(db_mapper_->EndEffectors.count(my_grasp.end_effector_id) == 0)
    {
      ROS_ERROR_STREAM("GraspsSerializer::parseGraspInfo : No end-effector present in the DB with ID " << my_grasp.end_effector_id);
      return false;
    }

    ROS_ASSERT(current_grasp["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
    my_grasp.grasp_name = std::string( current_grasp["name"] );

    // check if combination of object id, end effector id and grasp name already exists in the database
    std::map< grasp_id, std::tuple<object_id, endeffector_id, std::string> >::iterator it = db_mapper_->Grasps.begin();
    std::tuple<object_id, endeffector_id, std::string> grasp_info( object_id_, my_grasp.end_effector_id, my_grasp.grasp_name ) ;

    while(it != db_mapper_->Grasps.end() )
    {
        bool found = (it->second == grasp_info);
        if(found)
            ROS_WARN_STREAM("Grasp with name " << my_grasp.grasp_name << " for object " << object_id_ << " by end-effector " << my_grasp.end_effector_id << " already exists in database " << object_name_ << ".db !" );
        ++it;
    }

    ROS_ASSERT(current_grasp["waypoints"].getType() == XmlRpc::XmlRpcValue::TypeArray);

    int n_waypoints = current_grasp["waypoints"].size();
    ROS_DEBUG_STREAM("Serialize " << n_waypoints << " waypoints of grasp " << my_grasp.grasp_name << " for object "<< object_name_ << " and for end effector id " << my_grasp.end_effector_id);

    for( int j = 0; j < n_waypoints; ++j )
    {
      ROS_ASSERT(current_grasp["waypoints"][j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      my_grasp.waypoints.push_back( ros::Time( current_grasp["waypoints"][j] ) );
    }

    ROS_ASSERT(current_grasp["post"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    my_grasp.post_grasp = ros::Time( current_grasp["post"] );

    rec_grasps_.push_back(my_grasp);
  }

  if(rec_grasps_.empty())
  {
    ROS_ERROR_STREAM("GraspsSerializer::parseGraspInfo : grasps vector is empty!");
    return false;
  }
  
  grasp_info_set_ = true;
  
  return true;
}

bool GraspsSerializer::updateDatabase()
{
  if(!grasp_info_set_)
  {
    ROS_WARN_STREAM("GraspsSerializer::updateDatabase : grasp_info not set yet, call parseGraspInfo() first!");
    return false;
  }

  // traverse all bag files of the dataset where grasps were recorded
  std::string path = ros::package::getPath( dataset_.c_str() );

  // NOTE: the following is particual to the structure of the pacman_wp2_db
  
  // sub-folder
  path.append("/data/");

  // traverse all grasps
  for( unsigned int i = 0; i < rec_grasps_.size(); ++i)
  {
    // open bag file
    rosbag::Bag bag;
    rec_grasp my_grasp;
    my_grasp = rec_grasps_.at(i);
    std::string bagfile( path + object_name_ + "_" + my_grasp.grasp_name + "/" + "record.bag" );

    ROS_INFO_STREAM( "Opening the bag file: " << bagfile );

    bag.open( bagfile.c_str(), rosbag::bagmode::Read );

    // get the tf tree in one single variable
    std::string topic("tf");

    rosbag::View bag_view( bag, rosbag::TopicQuery( topic ) );
    foreach(rosbag::MessageInstance const m, bag_view)
    {
      tf2_msgs::TFMessage::ConstPtr tf_msg = m.instantiate<tf2_msgs::TFMessage>();

      // std::cout << *tf_msg << std::endl;

      foreach(geometry_msgs::TransformStamped const tfs, tf_msg->transforms)
      {
        tf::StampedTransform stampedTF;
        tf::transformStampedMsgToTF(tfs, stampedTF);
        //setTransform returns true!
        transformer_.setTransform(stampedTF);
      }
    }


    // set the tf variables
    std::string err_msg;
    tf::StampedTransform temp_tf;
    tf::StampedTransform ref_tf;

    // extract waypoints tf
    for(int w = 0; w < my_grasp.waypoints.size(); ++w )
    {
      geometry_msgs::Pose temp_tf_msg;
      if( transformer_.canTransform( world_tf_.c_str(), object_tf_.c_str(), my_grasp.waypoints.at(w) ) )
      {
        // set the first waypoint as the reference fixed frame
        if( w == 0 )
        {
          // get the initial object pose w.r.t. world at the first waypoint
          transformer_.lookupTransform( world_tf_.c_str(), object_tf_.c_str(), my_grasp.waypoints.at(w), ref_tf );
        }

        // get the hand w.r.t. world
        transformer_.lookupTransform( world_tf_.c_str(), hand_tf_.c_str(), my_grasp.waypoints.at(w), temp_tf );

        // set the waypoint hand w.r.t. the initial object
        tf::poseTFToMsg(ref_tf.inverse()*temp_tf, temp_tf_msg);
        ROS_DEBUG_STREAM("Extracted waypoint tf: " << w << temp_tf_msg);
        my_grasp.waypoints_obj_T_hand.push_back( temp_tf_msg );
      }
      else
      {
          ROS_WARN_STREAM("COULD NOT OBTAIN THE TRANFORM SPECIFIED BY THE WAYPOINT # " << w << "AT " << my_grasp.waypoints.at(w));
      }
    }

    // extract post-grasp tf
    if( transformer_.canTransform( world_tf_.c_str(), hand_tf_.c_str(), my_grasp.post_grasp ) )
    {
      // get the final hand pose w.r.t. world
      transformer_.lookupTransform( world_tf_.c_str(), hand_tf_.c_str(), my_grasp.post_grasp, ref_tf );

      // get the final object pose w.r.t. world
      transformer_.lookupTransform( world_tf_.c_str(), object_tf_.c_str(), my_grasp.post_grasp, temp_tf );

      // set the final object pose w.r.t. hand
      tf::poseTFToMsg(ref_tf.inverse()*temp_tf, my_grasp.post_hand_T_obj);
      ROS_DEBUG_STREAM("Extracted post grasp tf: " << my_grasp.post_hand_T_obj );
    }
    else
    {
        ROS_WARN_STREAM("COULD NOT OBTAIN THE TRANFORM SPECIFIED BY THE POST AT " << my_grasp.post_grasp);
    }

    // save current grasp to database
    int grasp_id = db_writer_->writeNewGrasp( object_id_, my_grasp.end_effector_id, my_grasp.grasp_name );
    if(grasp_id <= 0)
    {
      ROS_ERROR_STREAM( "Unable to write grasp " << my_grasp.grasp_name << " for object id " << object_id_ << " and end-effector id " << my_grasp.end_effector_id << " in database " << object_name_ << ".db !");
      return false;
    }

    if( !serializeGrasp( (uint)grasp_id, my_grasp ) )
    {
      ROS_WARN_STREAM("GraspsSerializer::save_recording : unable to serialize grasp - deleting database entry just created (grasp #" << grasp_id << ")");
      if( !db_writer_->deleteGrasp( (uint)grasp_id ) )
      {
        ROS_ERROR_STREAM("GraspsSerializer::save_recording : unable to delete grasp entry (grasp #" << grasp_id << ") from DB - consider deleting it by hand!");
        return false;
      }
    }
  }

  ROS_INFO_STREAM("The database " << object_name_ << ".db has been updated with " << rec_grasps_.size() << " grasps!");
  return true;
}

// private

void GraspsSerializer::openDB(std::string db_name)
{
  db_mapper_ = boost::shared_ptr<databaseMapper>(new databaseMapper(db_name));
  db_writer_ = boost::shared_ptr<databaseWriter>(new databaseWriter(db_name));
}

bool GraspsSerializer::serializeGrasp(uint grasp_id, const rec_grasp grasp)
{
  ROS_INFO_STREAM("Serializing...");
  
  // the message to serialize
  dual_manipulation_shared::grasp_trajectory grasp_msg;
  
  // create an object for grasping
  moveit_msgs::AttachedCollisionObject& attached_object = grasp_msg.attObject;
  trajectory_msgs::JointTrajectory& grasp_trajectory = grasp_msg.grasp_trajectory;

  trajectory_msgs::JointTrajectoryPoint traj_point;
  
  ROS_WARN_STREAM("GraspsSerializer::serialize_data : considering hand synergy from 0.0 (open) to 1.0 (close) -- change when this value is recorded as well");
  // hand: only open to closed
  traj_point.positions.push_back( 0.0 );
  grasp_trajectory.points.push_back( traj_point );
  traj_point.positions.clear();
  traj_point.positions.push_back( 1.0 );
  grasp_trajectory.points.push_back( traj_point );
  
  attached_object.object.id = std::get<0>( db_mapper_->Objects.at( object_id_ ) );
  
  // ASK HAMAL ABOUT THIS
  grasp_msg.object_db_id = object_id_;

  std::string ee = std::get<0>(db_mapper_->EndEffectors.at( grasp.end_effector_id ));
  
  if( ee == "left_hand" || ee == "right_hand" )
  {
    attached_object.link_name = ee + "_palm_link";
    grasp_trajectory.joint_names.clear();
    grasp_trajectory.joint_names.push_back( ee + "_synergy_joint" );
  }
  else if( ee == "table" )
  {
    attached_object.link_name = "world";
    grasp_trajectory.joint_names.clear();
  }
  else
  {
    ROS_FATAL_STREAM("GraspsSerializer::serialize_data : only considering left_hand, right_hand, and table for now - check the code for correctness if more end-effectors are added!!!");
    abort();
  }

  // the frame where the object position is considered (only when inserted)
  attached_object.object.header.frame_id = attached_object.link_name;
  attached_object.object.mesh_poses.clear();
  attached_object.object.mesh_poses.push_back( grasp.post_hand_T_obj );
  
  grasp_msg.ee_pose.clear();
  grasp_msg.ee_pose = grasp.waypoints_obj_T_hand;
  
  // finally, serialize the obtained grasp with serialize_ik() from dual manipulation shared
  bool ok = serialize_ik(grasp_msg, "object" + std::to_string(object_id_) + "/grasp" + std::to_string(grasp_id));
  
  if( ok )
  {
    ROS_INFO_STREAM( "Serialization object" + std::to_string( object_id_ ) << "/grasp" << grasp_id << " OK!" );
  }
  else
  {
    std::string temp = std::to_string( object_id_ );
    ROS_ERROR("In serialization object%s/grasp%d ! ", temp.c_str(), grasp_id);
  }
  return ok;
}
