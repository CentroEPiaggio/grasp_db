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

#ifndef GMU_H
#define GMU_H

#include <iostream>
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <string>
#include <thread>
#include "ros/package.h"
#include "dual_manipulation_shared/grasp_trajectory.h"
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/JointState.h"
#include "dual_manipulation_shared/databasemapper.h"
#include "dual_manipulation_shared/databasewriter.h"
#include <interactive_markers/interactive_marker_server.h>
#include <string>
#include <atomic>

class GMU
{
public:
    GMU();
    ~GMU();
    void publish_object();
    void publish_hands();
    void set_object(int id);
    void set_hands(std::vector<geometry_msgs::Pose> hands, geometry_msgs::Pose final_hand);
    void get_object(geometry_msgs::Pose& obj,geometry_msgs::Pose& final_obj);
    void get_hands(std::vector<geometry_msgs::Pose>& hands, geometry_msgs::Pose& final_hand);
    geometry_msgs::Pose get_wp(int i);
    void set_wp(int i, geometry_msgs::Pose wp);
    void delete_wp(int i);
    void add_extra_wp(int i);
    void clear();
    void setCurrentWaypoint(int wp);
    void toggle_objects_interaction(bool on);
    void force_im_update();

    boost::shared_ptr<databaseMapper> db_mapper;
    boost::shared_ptr<databaseWriter> db_writer;
    
private:
    void update_position(const visualization_msgs::Marker &marker_);
    void im_callback(const visualization_msgs::InteractiveMarkerFeedback& feedback);
    void publishTF(const sensor_msgs::JointState &msg);
    
    interactive_markers::InteractiveMarkerServer server;

    ros::NodeHandle node;    
    ros::Subscriber obj_sub;
    ros::Subscriber hand_sub;
    ros::Publisher object_marker_pub;
    ros::Publisher hands_marker_pub;
    ros::Subscriber im_sub_obj;
    ros::Subscriber im_sub_hand;
    ros::Subscriber js_sub;
    ros::Publisher fake_im_pub;
    
    geometry_msgs::Pose obj_pose;
    int obj_id;
    int current_waypoint=0;
    int number_of_waypoints=0;
    std::vector<geometry_msgs::Pose> hand_poses;
    geometry_msgs::Pose obj_final_pose;
    geometry_msgs::Pose hand_final_pose;
    // this variable will store a marker for the post-grasp object position
    visualization_msgs::Marker object_marker;

    tf::Transform transform_;
    tf::TransformBroadcaster tf_broadcaster_;

    std::string db_name_;
    std::atomic_bool objects_active;
    std::string hand_mesh_path_;
    double hand_mesh_scale_;
    std::string ee_link_name_;
};

#endif //GMU_H