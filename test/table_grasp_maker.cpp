#include <ros/ros.h>
#include <kdl_conversions/kdl_msg.h>

#include "table_grasp_maker.h"
#include "dual_manipulation_shared/serialization_utils.h"

tableGraspMaker::tableGraspMaker(std::string db_name):db_writer(db_name), db_mapper(db_name)
{
  
}

bool tableGraspMaker::read_data_from_file(std::string& obj_name, std::string& grasp_name, KDL::Frame& obj_ee_final, std::string filename)
{
  std::string path = ros::package::getPath("dual_manipulation_grasp_db");
  path.append(filename);
  
  std::fstream fs;
  // fs.open (path, std::fstream::in | std::fstream::out | std::fstream::app*/);
  fs.open (path, std::fstream::in);
  if (fs.fail())
    return false;
  
  double obj_grasp_data[6];
  
  fs >> obj_name;
  fs >> grasp_name;
  for (int i=0; i<6; ++i)
    fs >> obj_grasp_data[i];
  
  fs.close();
  
  for (int i=0; i<3; i++)
    obj_ee_final.p.data[i] = obj_grasp_data[i];
  obj_ee_final.M = KDL::Rotation::RPY(obj_grasp_data[3],obj_grasp_data[4],obj_grasp_data[5]);

  return true;
}

bool tableGraspMaker::serialize_data(const dual_manipulation_shared::grasp_trajectory& grasp_msg, int object_id, int grasp_id)
{
  // save the obtained grasp
  if(serialize_ik(grasp_msg,"object" + std::to_string(object_id) + "/grasp" + std::to_string(grasp_id)))
  {
    ROS_INFO_STREAM("Serialization object" + std::to_string(object_id) << "/grasp" + std::to_string(grasp_id) << " OK!");
  }
  else
  {
    ROS_ERROR_STREAM("In serialization object" + std::to_string(object_id) << "/grasp" + std::to_string(grasp_id));
    return false;
  }
  
  return true;
}

void tableGraspMaker::build_grasp_msg(dual_manipulation_shared::grasp_trajectory& grasp_msg, const KDL::Frame& obj_ee_frame, int obj_id, std::string ee_frame_name)
{
  // create an object for grasping
  moveit_msgs::AttachedCollisionObject& attached_object = grasp_msg.attObject;
  u_int64_t& object_db_id = grasp_msg.object_db_id;
  std::vector<geometry_msgs::Pose>& ee_pose = grasp_msg.ee_pose;
  
  // // NOTE: this commented part could be useful when building grasps for hands
  // trajectory_msgs::JointTrajectory& grasp_trajectory = grasp_msg.grasp_trajectory;
  // trajectory_msgs::JointTrajectoryPoint traj_point;
  // // hand: only open to closed
  // traj_point.positions.push_back(0.0);
  // grasp_trajectory.points.push_back(traj_point);
  // traj_point.positions.clear();
  // traj_point.positions.push_back(1.0);
  // grasp_trajectory.points.push_back(traj_point);
  // std::string ee = ee_name_map.at(end_effector_id);
  // if(ee == "left_hand" || ee == "right_hand")
  // {
  //   attached_object.link_name = ee + "_palm_link";
  //   grasp_trajectory.joint_names.clear();
  //   grasp_trajectory.joint_names.push_back(ee + "_synergy_joint");
  // }
  // else
  
  attached_object.link_name = ee_frame_name;

  // this will be used to read the DB
  object_db_id = obj_id;

  geometry_msgs::Pose obj_ee_final,obj_ee_wp,ee_obj;
  tf::poseKDLToMsg(obj_ee_frame,obj_ee_final);
  tf::poseKDLToMsg(obj_ee_frame.Inverse(),ee_obj);
  
  // add a higher waypoint on the table
  KDL::Frame Tz(KDL::Frame(KDL::Vector(0,0,waypoint_height_)));
  KDL::Frame ee_obj_frame(obj_ee_frame.Inverse());
  tf::poseKDLToMsg((Tz*ee_obj_frame).Inverse(),obj_ee_wp);
  
  // the frame where the object position is considered (only when inserted)
  attached_object.object.header.frame_id = attached_object.link_name;
  attached_object.object.mesh_poses.clear();
  attached_object.object.mesh_poses.push_back(ee_obj);
  
  // add the waypoint and the final pose as end-effector poses
  ee_pose.clear();
  ee_pose.push_back(obj_ee_wp);
  ee_pose.push_back(obj_ee_final);
}

bool tableGraspMaker::create_table_grasps(int obj_id, std::string grasp_name, KDL::Frame obj_ee)
{
  dual_manipulation_shared::grasp_trajectory grasp_msg;
  
  // for the desired amount of steps, rotate around global z axis
  for(int i=0; i<yaw_steps_; i++)
  {
    KDL::Frame rotz(KDL::Rotation::RotZ(M_PI*2*i/yaw_steps_));
    KDL::Frame obj_ee_rotated(obj_ee);
    
    std::string grasp_name_rotated(grasp_name);
    grasp_name_rotated += "_" + std::to_string(2*180*i/yaw_steps_);
    
    obj_ee_rotated.M = rotz.M*obj_ee_rotated.M;
    
    // build grasp msg for this orientation
    build_grasp_msg(grasp_msg, obj_ee_rotated, obj_id, end_effector_frame_);

    // write its entry in the database
    int grasp_id = db_writer.writeNewGrasp(obj_id,end_effector_id_,grasp_name_rotated);
    if(grasp_id == -1)
    {
      ROS_ERROR_STREAM("Unable to write entry in the grasp DB - returning");
      return false;
    }

    // serialize it (using the id just obtained)
    if(!serialize_data(grasp_msg, obj_id, grasp_id))
    {
      ROS_ERROR_STREAM("Unable to serialize grasp message - returning...");
      if(!db_writer.deleteGrasp(grasp_id))
      {
	ROS_ERROR_STREAM("Unable to delete entry from DB: consider deleting it by hand!");
      }
      return false;
    }
  }
  
  return true;
}
