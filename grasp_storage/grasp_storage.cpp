#include "grasp_storage.h"
#include <sqlite3.h>
#include "dual_manipulation_shared/grasp_trajectory.h"
#include "dual_manipulation_shared/serialization_utils.h"

#define ABSURD_TIME 100.0
#define THREAD_TIME 10000

std::string str_quotesql( const std::string& s ) {
  return std::string("'") + s + std::string("'");
}

std::string int_quotesql( const int& s ) {
  return std::string("'") + std::to_string(s) + std::string("'");
}

bool grasp_storage::get_transform(tf::StampedTransform& result, std::string target, std::string source, double wait_time)
{
  std::string err_msg;
  
  tf::StampedTransform target_T_source;
  
  if(!tf.waitForTransform(target,source,ros::Time(0), ros::Duration(wait_time), ros::Duration(0.01), &err_msg))
  {
    ROS_ERROR("Error in tf: %s",err_msg.c_str());
    target_T_source.setIdentity();
    return false;
  }
  else
    tf.lookupTransform(target,source, ros::Time(0), target_T_source);
  
  result=target_T_source;
  
  return true;
}

grasp_storage::grasp_storage(std::string db_name)
{
  db_mapper_ = boost::shared_ptr<databaseMapper>(new databaseMapper(db_name));
  db_writer_ = boost::shared_ptr<databaseWriter>(new databaseWriter(db_name));
}

void grasp_storage::reset_grasp_storage()
{
  obj0_trajectory.clear();
  hand_objF = geometry_msgs::Pose();
  world_T_obj0 = tf::StampedTransform();
  
  grasp_info_set = false;
}

bool grasp_storage::set_grasp_info(int object_id_, int end_effector_id_, std::string grasp_name_, std::vector< ros::Time > snapshots_, std::string world_tf_, std::string hand_tf_, std::string object_tf_)
{
  if(!stop_thread)
  {
    ROS_WARN_STREAM("grasp_storage::set_grasp_info : cannot set grasp_info while thread is running!");
    return false;
  }
  
  if(db_mapper_->Objects.count(object_id_) == 0)
  {
    ROS_ERROR_STREAM("grasp_storage::set_grasp_info : No object present in the DB with ID " << object_id_);
    return false;
  }
  
  if(db_mapper_->EndEffectors.count(end_effector_id_) == 0)
  {
    ROS_ERROR_STREAM("grasp_storage::set_grasp_info : No end-effector present in the DB with ID " << end_effector_id_);
    return false;
  }
  
  if(snapshots_.empty())
  {
    ROS_ERROR_STREAM("grasp_storage::set_grasp_info : snapshots vector is empty!");
    return false;
  }
  
  object_id = object_id_;
  end_effector_id = end_effector_id_;
  grasp_name = grasp_name_;
  
  world_tf = world_tf_;
  hand_tf = hand_tf_;
  object_tf = object_tf_;
  
  snapshots.clear();
  snapshots = snapshots_;
  
  grasp_info_set = true;
  
  return true;
}

bool grasp_storage::record_grasp()
{
  ROS_INFO_STREAM("grasp_storage::record_grasp : start recording trajectory");
  
  if(!grasp_info_set)
  {
    ROS_WARN_STREAM("grasp_storage::record_grasp : grasp_info not set yet, call set_grasp_info() first!!!");
    return false;
  }
  
  if(stop_thread)
  {
    usleep(2*THREAD_TIME);
    save_start = true;
    stop_thread = false;
    tf_periodic_listener = boost::shared_ptr<std::thread>(new std::thread(&grasp_storage::thread_body, this));
  }
  else
    ROS_WARN_STREAM("grasp_storage::record_grasp : thread was already running - not starting again");
  
  return true;
}

bool grasp_storage::save_recording()
{
  bool ok = true;
  if(!grasp_info_set)
  {
    ROS_ERROR_STREAM("grasp_storage::save_recording : grasp_info not set yet, call set_grasp_info() first!!!");
    ok = false;
  }
  if(!stop_thread)
  {
    ROS_ERROR_STREAM("grasp_storage::save_recording : thread is running, wait for completion!!!");
    ok = false;
  }
  if(obj0_trajectory.empty())
  {
    ROS_ERROR_STREAM("grasp_storage::save_recording : trajectory to save is empty - returning!!!");
    ok = false;
  }
  if(!ok)
  {
    reset_grasp_storage();
    return false;
  }
  
  int new_grasp_id = save_in_db(object_id,end_effector_id,grasp_name);
  if(new_grasp_id <= 0)
  {
    reset_grasp_storage();
    return false;
  }
  
  bool ser_ok = serialize_data((uint)new_grasp_id);
  
  if(!ser_ok)
  {
    ROS_WARN_STREAM("grasp_storage::save_recording : unable to serialize grasp file - deleting database entry just created (grasp #" << new_grasp_id << ")");
    bool del_ok = db_writer_->deleteGrasp((uint)new_grasp_id);
    if(!del_ok)
      ROS_ERROR_STREAM("grasp_storage::save_recording : unable to delete grasp entry (grasp #" << new_grasp_id << ") from DB - consider deleting it by hand!");
  }
  
  reset_grasp_storage();
  return ser_ok;
}

void grasp_storage::stop_recording()
{
  stop_record_trajectory_pose();
  reset_grasp_storage();
}

bool grasp_storage::save_start_pose()
{
  ROS_INFO_STREAM("saving start object position...");
  if (get_transform(world_T_obj0,world_tf,object_tf,100.0))
  {
    ROS_INFO_STREAM("saved!");
    return true;
  }
  else
  {
    ROS_ERROR("could not start saving poses");
    return false;
  }
}

void grasp_storage::single_step(bool force_snapshot)
{
  geometry_msgs::Pose obj0_traj;
  tf::StampedTransform world_T_traj;
  if(save_start)
  {
    if(!save_start_pose())
      abort();
    save_start = false;
    return;
  }
  if (get_transform(world_T_traj,world_tf,hand_tf))
  {
    // std::cout<<world_T_traj.stamp_<<std::endl;
    tf::poseTFToMsg(world_T_obj0.inverse()*world_T_traj,obj0_traj);
    if (force_snapshot)
    {
      obj0_trajectory.push_back(obj0_traj);
      return;
    }
    for(auto& snapshot_time:snapshots)
    {
      double diff = fabs((world_T_traj.stamp_ - snapshot_time).toSec());
      if (diff < 0.03)
      {
	obj0_trajectory.push_back(obj0_traj);
	snapshot_time = ros::Time(ABSURD_TIME);
	std::cout<<"SAVED snapshot"<<std::endl;
	save_end_pose();
      }
    }
    
    // check exit condition : finished last snapshot!
    if(snapshots.back() == ros::Time(ABSURD_TIME))
    {
      stop_record_trajectory_pose();
    }
  }
}

void grasp_storage::thread_body()
{
  while(!stop_thread)
  {
    single_step(false);
    usleep(THREAD_TIME);
  }
  ROS_INFO_STREAM("stop recording trajectory");
}

void grasp_storage::stop_record_trajectory_pose()
{
  stop_thread = true;
  //usleep(100000);
  //delete tf_periodic_listener;
}

void grasp_storage::save_end_pose()
{
  ROS_INFO_STREAM("saving end position...");
  
  tf::StampedTransform world_T_hand, world_T_obj;
  get_transform(world_T_hand,world_tf,hand_tf);
  
  get_transform(world_T_obj,world_tf,object_tf);
  tf::poseTFToMsg(world_T_hand.inverse()*world_T_obj,hand_objF);
  
  ROS_INFO_STREAM("saved!");
}

bool grasp_storage::serialize_data(uint new_grasp_id)
{
  ROS_INFO_STREAM("serializing...");
  
  dual_manipulation_shared::grasp_trajectory grasp_msg;
  
  // create an object for grasping
  moveit_msgs::AttachedCollisionObject& attached_object = grasp_msg.attObject;
  trajectory_msgs::JointTrajectory& grasp_trajectory = grasp_msg.grasp_trajectory;
  trajectory_msgs::JointTrajectoryPoint traj_point;
  
  ROS_WARN_STREAM("grasp_storage::serialize_data : only considering hand synergy from 0.0 to 1.0 - change when more data will be available!!!");
  // hand: only open to closed
  traj_point.positions.push_back(0.0);
  grasp_trajectory.points.push_back(traj_point);
  traj_point.positions.clear();
  traj_point.positions.push_back(1.0);
  grasp_trajectory.points.push_back(traj_point);
  
  attached_object.object.id = std::get<0>(db_mapper_->Objects.at(object_id));
  
  grasp_msg.object_db_id = 1;
  
  // grasp counter!
  int counter = 0;
  
  std::string ee = std::get<0>(db_mapper_->EndEffectors.at(end_effector_id));
  
  if(ee == "left_hand" || ee == "right_hand")
  {
    attached_object.link_name = ee + "_palm_link";
    grasp_trajectory.joint_names.clear();
    grasp_trajectory.joint_names.push_back(ee + "_synergy_joint");
  }
  else if(ee == "table")
  {
    attached_object.link_name = "world";
    grasp_trajectory.joint_names.clear();
  }
  else
  {
    ROS_FATAL_STREAM("grasp_storage::serialize_data : only considering left_hand, right_hand, and table for now - check the code for correctness if more end-effectors are added!!!");
    abort();
  }
  
  // the frame where the object position is considered (only when inserted)
  attached_object.object.header.frame_id = attached_object.link_name;
  attached_object.object.mesh_poses.clear();
  attached_object.object.mesh_poses.push_back(hand_objF);
  
  grasp_msg.ee_pose.clear();
  
  obj0_trajectory.pop_back();
  for (auto item:obj0_trajectory)
    grasp_msg.ee_pose.push_back(item);
  
  // save the obtained grasp
  bool ok = serialize_ik(grasp_msg,"object" + std::to_string(object_id) + "/grasp" + std::to_string(new_grasp_id));
  
  if(ok)
  {
    ROS_INFO_STREAM("Serialization object" + std::to_string(object_id) << "/grasp" << new_grasp_id << " OK!");
  }
  else
  {
    std::string temp = std::to_string(object_id);
    ROS_ERROR("In serialization object%s/grasp%d ! ", temp.c_str(), new_grasp_id);
  }
  
  return ok;
}

int grasp_storage::save_in_db(int obj_id, int ee_id, std::string name)
{
  ROS_INFO_STREAM("saving in db...");
  int scoreID = db_writer_->writeNewGrasp(obj_id, ee_id, name);
  if(scoreID <= 0)
  {
    ROS_ERROR_STREAM("grasp_storage::save_in_db : unable to save new grasp " << grasp_name << " for object id " << obj_id << " and end-effector id " << ee_id);
  }
  
  return scoreID;
}

grasp_storage::~grasp_storage()
{
  //delete tf_periodic_listener;
}