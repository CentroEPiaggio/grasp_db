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

#ifndef GRASPS_SERIALIZER_H
#define GRASPS_SERIALIZER_H

#include <iostream>
#include <string>
#include <thread>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "dual_manipulation_shared/databasemapper.h"
#include "dual_manipulation_shared/databasewriter.h"

class GraspsSerializer
{
public:
  GraspsSerializer(){};

  bool parseGraspInfo(XmlRpc::XmlRpcValue grasps);
    
  bool updateDatabase();
 
  ~GraspsSerializer() {};
  
private:

  boost::shared_ptr<databaseMapper> db_mapper_;
  boost::shared_ptr<databaseWriter> db_writer_;

  // grasp info
  int object_id_;
  std::string object_name_, dataset_;
  struct rec_grasp
  {
    int end_effector_id;
    std::string grasp_name;
    std::vector<ros::Time> waypoints;
    ros::Time post_grasp;
    std::vector<geometry_msgs::Pose> waypoints_obj_T_hand;
    geometry_msgs::Pose post_hand_T_obj;
  };
  std::vector<rec_grasp> rec_grasps_;
  std::string world_tf_, hand_tf_, object_tf_;

  // control parameters
  bool grasp_info_set_ = false;

  // holder of the whole tf tree
  tf::Transformer transformer_;
  
  void openDB(std::string db_name);
  bool serializeGrasp(uint grasp_id, const rec_grasp grasp);
};

#endif //GRASPS_SERIALIZER_H
