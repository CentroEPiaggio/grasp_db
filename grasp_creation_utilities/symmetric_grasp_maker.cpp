#include "grasp_creation_utilities/symmetric_grasp_maker.h"
#include <kdl_conversions/kdl_msg.h>
#include "dual_manipulation_shared/serialization_utils.h"

#define CLASS_NAMESPACE "symmetricGraspMaker::"
#define DEBUG 1

symmetricGraspMaker::symmetricGraspMaker(uint ee_id, std::string ee_frame, const std::vector< std::string >& new_joint_names, std::string db_name)
{
    end_effector_id_ = ee_id;
    end_effector_frame_ = ee_frame;
    joint_names_ = new_joint_names;
    db_name_ = db_name;
}

bool symmetricGraspMaker::transform_grasp(uint obj_id, uint grasp_id, std::string new_grasp_base_name, std::vector<uint> new_grasp_ids, uint how_many_rot, KDL::Vector rot_axis, KDL::Frame rotFrame_obj = KDL::Frame::Identity())
{
    if(!new_grasp_ids.empty() && new_grasp_ids.size() != how_many_rot-1)
    {
        ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : the number of new grasp ids (" << new_grasp_ids.size() << ") should be equal to (how_many_rot-1)=" << how_many_rot-1);
        return false;
    }
    
    dual_manipulation_shared::grasp_trajectory grasp_msg;
    
    // try to read the grasp
    if(!read_grasp_msg(obj_id,grasp_id,grasp_msg))
    {
        ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : unable to read grasp #" << grasp_id << " for object #" << obj_id);
        return false;
    }
    
    // I want to be sure to have an updated writer every time, so I use a shared_ptr
    boost::shared_ptr<databaseWriter> db_writer(new databaseWriter(db_name_));
    assert(db_writer.unique());
    
    bool write_ok = true;
    bool delete_ok = true;
    int i;
    
    for(i=1; i<how_many_rot; i++)
    {
        std::string new_grasp_name(new_grasp_base_name);
        new_grasp_name += "_" + std::to_string(2*180*i/how_many_rot);
        
        // write a new entry in the DB
        int writer_ret;
        uint new_grasp_id = 0;
        if(!new_grasp_ids.empty())
            new_grasp_id = new_grasp_ids.at(i-1);
        if(new_grasp_id == 0)
            writer_ret = db_writer->writeNewGrasp(obj_id,end_effector_id_,new_grasp_name);
        else
            writer_ret = db_writer->writeNewGrasp(new_grasp_id,obj_id,end_effector_id_,new_grasp_name);
        if(writer_ret < 0)
            break;
        
        new_grasp_id = (uint)writer_ret;
        
        #if DEBUG
        std::cout << CLASS_NAMESPACE << __func__ << " : transforming grasp #" << grasp_id << " >> #" << new_grasp_id << " (object " << obj_id << " - new_grasp_name " << new_grasp_name << ")" << std::endl;
        #endif
        
        KDL::Frame rot_mat(KDL::Rotation::Rot(rot_axis,M_PI*2*i/how_many_rot));
        rot_mat = rot_mat*rotFrame_obj;
        transform_grasp_symmetry(grasp_msg,obj_id,end_effector_frame_,joint_names_,rot_mat);
        
        write_ok = (write_grasp_msg(obj_id,new_grasp_id,grasp_msg) > 0);
        if(!write_ok)
        {
            ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : Unable to serialize the new grasp - performing delete action on the DB entry just created...");
            delete_ok = db_writer->deleteGrasp(new_grasp_id);
            if(!delete_ok)
                ROS_ERROR_STREAM(CLASS_NAMESPACE << __func__ << " : Unable to reserialize the grasp and to delete the entry just created!!! Consider deleting grasp #" << new_grasp_id << " by hand!!!");
            else
                ROS_INFO_STREAM(CLASS_NAMESPACE << __func__ << " : Successfully deleted new grasp (" << new_grasp_id << ") entry from DB");
            break;
        }
    }
    
    if(i < how_many_rot)
        ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : only serialized " << (i-1) << " grasps instead of the desired " << (how_many_rot-1));
    return (write_ok && delete_ok);
}

void symmetricGraspMaker::transform_premultiply(std::vector<KDL::Frame>& poses, KDL::Frame frame)
{
    for(int i=0; i<poses.size(); i++)
        poses.at(i) = frame*poses.at(i);
}

void symmetricGraspMaker::transform_grasp_symmetry(dual_manipulation_shared::grasp_trajectory& grasp_msg, uint obj_id, std::string new_link_name, const std::vector< std::string >& new_joint_names, KDL::Frame rotFrame_obj)
{
    // first of all, convert all poses to KDL, renormalizing the quaternion
    std::vector<KDL::Frame> grasp_frames;
    for(auto& p:grasp_msg.ee_pose)
    {
        geometry_msgs::Quaternion& q(p.orientation);
        double q_norm;
        q_norm = std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
        if(q_norm < 0.99 || q_norm > 1.01)
            ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : quaternion needed normalization (norm was " << q_norm << " | out of allowed range [0.98,1.02])");
        KDL::Frame tmp;
        tmp.p.x(p.position.x);
        tmp.p.y(p.position.y);
        tmp.p.z(p.position.z);
        tmp.M = KDL::Rotation::Quaternion(q.x/q_norm,q.y/q_norm,q.z/q_norm,q.w/q_norm);
        grasp_frames.push_back(tmp);
    }
    KDL::Frame postgrasp_frame;
    geometry_msgs::Pose& obj_pose(grasp_msg.attObject.object.mesh_poses.front());
    postgrasp_frame.p.x(obj_pose.position.x);
    postgrasp_frame.p.y(obj_pose.position.y);
    postgrasp_frame.p.z(obj_pose.position.z);
    geometry_msgs::Quaternion& q(obj_pose.orientation);
    double q_norm;
    q_norm = std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    if(q_norm < 0.99 || q_norm > 1.01)
        ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : quaternion needed normalization (norm was " << q_norm << " | out of allowed range [0.98,1.02])");
    postgrasp_frame.M = KDL::Rotation::Quaternion(q.x/q_norm,q.y/q_norm,q.z/q_norm,q.w/q_norm);
    postgrasp_frame = postgrasp_frame.Inverse();
    
    // transform using pre-multiplication
    transform_premultiply(grasp_frames,rotFrame_obj);
    std::vector<KDL::Frame> v({postgrasp_frame});
    transform_premultiply(v,rotFrame_obj);
    postgrasp_frame = v.front();
    
    // convert back to geometry_msgs
    grasp_msg.ee_pose.clear();
    for(auto& f:grasp_frames)
    {
        geometry_msgs::Pose p;
        tf::poseKDLToMsg(f,p);
        grasp_msg.ee_pose.push_back(p);
    }
    postgrasp_frame = postgrasp_frame.Inverse();
    tf::poseKDLToMsg(postgrasp_frame,obj_pose);
    
    grasp_msg.attObject.object.mesh_poses.clear();
    grasp_msg.attObject.object.mesh_poses.push_back(obj_pose);
    grasp_msg.attObject.link_name = new_link_name;
    grasp_msg.attObject.object.header.frame_id = grasp_msg.attObject.link_name;
    if(grasp_msg.object_db_id != obj_id)
    {
        ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : grasp_msg.object_db_id (" << grasp_msg.object_db_id << ") and obj_id (" << obj_id << ") differ! Proceeding assigning the new obj_id");
        grasp_msg.object_db_id = obj_id;
    }
    grasp_msg.grasp_trajectory.joint_names.clear();
    grasp_msg.grasp_trajectory.joint_names = new_joint_names;
    
    return;
}
