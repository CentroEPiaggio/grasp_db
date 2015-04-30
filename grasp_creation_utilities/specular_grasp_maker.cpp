#include "grasp_creation_utilities/specular_grasp_maker.h"
#include <kdl_conversions/kdl_msg.h>
#include "dual_manipulation_shared/serialization_utils.h"

specularGraspMaker::specularGraspMaker(uint ee_id, std::string ee_frame, const std::vector< std::string >& new_joint_names, std::string db_name)
{
  end_effector_id_ = ee_id;
  end_effector_frame_ = ee_frame;
  joint_names_ = new_joint_names;
  db_name_ = db_name;
}

bool specularGraspMaker::transform_grasp(uint obj_id, uint grasp_id, std::string new_grasp_name, bool top_bottom)
{
  dual_manipulation_shared::grasp_trajectory grasp_msg;
  
  // try to read the grasp
  if(!read_grasp_msg(obj_id,grasp_id,grasp_msg))
    return false;
  
  // I want to be sure to have an updated writer every time, so I use a shared_ptr
  boost::shared_ptr<databaseWriter> db_writer(new databaseWriter(db_name_));
  assert(db_writer.unique());
  
  // write a new entry in the DB
  int writer_ret = db_writer->writeNewGrasp(obj_id,end_effector_id_,new_grasp_name);
  if(writer_ret < 0)
    return false;
  
  uint new_grasp_id = (uint)writer_ret;
  
  transform_grasp_specularity(grasp_msg,obj_id,end_effector_frame_,joint_names_,top_bottom);
  
  bool write_ok = write_grasp_msg(obj_id,new_grasp_id,grasp_msg);
  bool delete_ok = true;
  if(!write_ok)
  {
    ROS_WARN_STREAM("specularGraspMaker::transform_grasp : Unable to serialize the new grasp - performing delete action on the DB entry just created...");
    delete_ok = db_writer->deleteGrasp(new_grasp_id);
    if(!delete_ok)
      ROS_ERROR_STREAM("specularGraspMaker::transform_grasp : Unable to reserialize the grasp and to delete the entry just created!!! Consider deleting grasp #" << new_grasp_id << " by hand!!!");
    else
      ROS_INFO_STREAM("specularGraspMaker::transform_grasp : Successfully deleted new grasp (" << new_grasp_id << ") entry from DB");
  }
  
  return (write_ok && delete_ok);
}

void specularGraspMaker::transform_specular_y(geometry_msgs::Pose& pose)
{
    KDL::Frame pose_tmp;
    tf::poseMsgToKDL(pose,pose_tmp);
    pose_tmp.p.data[1] = -pose_tmp.p.data[1];
    pose_tmp.M.data[1] = -pose_tmp.M.data[1];
    pose_tmp.M.data[3] = -pose_tmp.M.data[3];
    pose_tmp.M.data[5] = -pose_tmp.M.data[5];
    pose_tmp.M.data[7] = -pose_tmp.M.data[7];
    tf::poseKDLToMsg(pose_tmp,pose);
}

void specularGraspMaker::transform_specular_y(std::vector< geometry_msgs::Pose >& poses)
{
  for(int i=0; i<poses.size(); i++)
  {
    transform_specular_y(poses.at(i));
  }
}

void specularGraspMaker::transform_premultiply(std::vector<geometry_msgs::Pose>& poses, KDL::Frame frame)
{
  KDL::Frame pose_tmp;
  for(int i=0; i<poses.size(); i++)
  {
    tf::poseMsgToKDL(poses.at(i),pose_tmp);
    tf::poseKDLToMsg(frame*pose_tmp,poses.at(i));
  }
}

bool specularGraspMaker::read_grasp_msg(uint obj_id, uint grasp_id, dual_manipulation_shared::grasp_trajectory& grasp_msg)
{
  if(deserialize_ik(grasp_msg,"object" + std::to_string(obj_id) + "/grasp" + std::to_string(grasp_id)))
    std::cout << "Deserialization object" + std::to_string(obj_id) + "/grasp" + std::to_string(grasp_id) << " OK!" << std::endl;
  else
  {
    std::cout << "Error in deserialization object" + std::to_string(obj_id) + "/grasp" + std::to_string(grasp_id) << "!" << std::endl;
    return false;
  }
  return true;
}

bool specularGraspMaker::write_grasp_msg(uint obj_id, uint grasp_id, const dual_manipulation_shared::grasp_trajectory& grasp_msg)
{
  if(serialize_ik(grasp_msg,"object" + std::to_string(obj_id) + "/grasp" + std::to_string(grasp_id)))
    std::cout << "Serialization object" + std::to_string(obj_id) << "/grasp" << grasp_id << " OK!" << std::endl;
  else
  {
    std::cout << "Error in serialization object" + std::to_string(obj_id) << "/grasp" << grasp_id << "!" << std::endl;
    return false;
  }
  return true;
}

void specularGraspMaker::transform_grasp_specularity(dual_manipulation_shared::grasp_trajectory& grasp_msg, uint obj_id, std::string new_link_name, const std::vector< std::string >& new_joint_names, bool top_bottom)
{
  // transform using specularity on x|z plane (y is the orthogonal axis we consider)
  transform_specular_y(grasp_msg.ee_pose);
  
  // NOTE: transform top to bottom and side_low to side_high
  if(top_bottom)
    transform_premultiply(grasp_msg.ee_pose,KDL::Frame(KDL::Rotation::RotX(M_PI)));
  
  KDL::Frame obj_frame;
  geometry_msgs::Pose obj_pose;
  tf::poseMsgToKDL(grasp_msg.attObject.object.mesh_poses.front(),obj_frame);
  obj_frame = obj_frame.Inverse();
  tf::poseKDLToMsg(obj_frame,obj_pose);
  transform_specular_y(obj_pose);
  
  // NOTE: transform top to bottom and side_low to side_high
  if(top_bottom)
  {
    std::vector<geometry_msgs::Pose> poses_vec = {obj_pose};
    transform_premultiply(poses_vec,KDL::Frame(KDL::Rotation::RotX(M_PI)));
    obj_pose = poses_vec.front();
  }
  
  tf::poseMsgToKDL(obj_pose,obj_frame);
  obj_frame = obj_frame.Inverse();
  tf::poseKDLToMsg(obj_frame,obj_pose);
  grasp_msg.attObject.object.mesh_poses.clear();
  grasp_msg.attObject.object.mesh_poses.push_back(obj_pose);
  grasp_msg.attObject.link_name = new_link_name;
  grasp_msg.attObject.object.header.frame_id = grasp_msg.attObject.link_name;
  if(grasp_msg.object_db_id != obj_id)
  {
    ROS_WARN_STREAM("specularGraspMaker::reserialize_grasp : grasp_msg.object_db_id (" << grasp_msg.object_db_id << ") and obj_id (" << obj_id << ") differ! Proceeding assigning the new obj_id");
    grasp_msg.object_db_id = obj_id;
  }
  grasp_msg.grasp_trajectory.joint_names.clear();
  grasp_msg.grasp_trajectory.joint_names = new_joint_names;
  
  return;
}
