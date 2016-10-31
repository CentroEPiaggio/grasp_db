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

#include <ros/ros.h>
#include <kdl_conversions/kdl_msg.h>

#include "grasp_creation_utilities/table_grasp_maker.h"
#include "dual_manipulation_shared/serialization_utils.h"

tableGraspMaker::tableGraspMaker(std::string db_name, uint yaw_steps, uint ee_id, double wp_height, std::string ee_frame):db_writer(db_name), db_mapper(db_name)
{
    yaw_steps_ = yaw_steps;
    end_effector_id_ = ee_id;
    waypoint_height_ = wp_height;
    end_effector_frame_ = ee_frame;
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

bool tableGraspMaker::create_table_grasps(int obj_id, std::string grasp_name, KDL::Frame obj_ee, uint64_t new_grasp_id)
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
        int writer_ret;
        if(new_grasp_id == 0)
            writer_ret = db_writer.writeNewGrasp(obj_id,end_effector_id_,grasp_name_rotated);
        else
            writer_ret = db_writer.writeNewGrasp(new_grasp_id+i,obj_id,end_effector_id_,grasp_name_rotated);
        if(writer_ret == -1)
        {
            ROS_ERROR_STREAM("Unable to write entry in the grasp DB - returning");
            return false;
        }
        
        // serialize it (using the id just obtained)
        if(write_grasp_msg(obj_id,writer_ret,grasp_msg)<=0)
        {
            ROS_ERROR_STREAM("Unable to serialize grasp message - returning...");
            if(!db_writer.deleteGrasp(writer_ret))
            {
                ROS_ERROR_STREAM("Unable to delete entry from DB: consider deleting it by hand!");
            }
            return false;
        }
    }
    
    return true;
}
