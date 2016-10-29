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

#ifndef TABLE_GRASP_MAKER_H
#define TABLE_GRASP_MAKER_H

#include <string>
#include <kdl/frames.hpp>

#include "dual_manipulation_shared/databasemapper.h"
#include "dual_manipulation_shared/databasewriter.h"
#include "dual_manipulation_shared/grasp_trajectory.h"

// default values
#define END_EFFECTOR_ID 3		// the ID of the end-effector (table) to consider
#define END_EFFECTOR_FRAME "world"	// the frame to be used in the request
#define YAW_STEPS 16			// how many steps to use when rotating the grasp
#define WAYPOINT_HEIGHT 0.1		// height of the waypoint used for pre-ungrasp and post-grasp

class tableGraspMaker
{
public:
  
  /**
   * @brief Constructor, uses default values if other values are not provided
   * 
   * @param db_name name of the database to use
   * @param yaw_steps how many times to replicate the grasp, rotating around global z-axis
   * @param ee_id the id of the end-effector (table) to use for saving the grasps
   * @param wp_height height of the wp over the table (for the table pre-grasp)
   * @param ee_frame (if different from world) the frame to write in the attObject frame field of the grasp: says to which frame the object will be attached, depends on the end-effector
   */
  tableGraspMaker(std::string db_name = "test.db", uint yaw_steps = YAW_STEPS, uint ee_id = END_EFFECTOR_ID, double wp_height = WAYPOINT_HEIGHT, std::string ee_frame = END_EFFECTOR_FRAME);
  
  bool read_data_from_file(std::string& obj_name, std::string& grasp_name, KDL::Frame& obj_ee_final, std::string filename = "/test/table_grasp_data.txt");
  
  /**
   * @brief Create table grasps: the number is specified in the constructor
   * 
   * @param obj_id id of the object
   * @param grasp_name the prefix of the name to give to the new grasps
   * @param obj_ee the frame to consider between object and end-effector - rotations will be applied about the object z-axis
   * @param new_grasp_id the first id to use to make new grasps - default: write in the database and get the id from there
   * @param ec_id environment constraint id - zero by default (meaningless - could not be used with new transition types)
   */
  bool create_table_grasps(int obj_id, std::string grasp_name, KDL::Frame obj_ee, uint64_t new_grasp_id=0, constraint_id ec_id = 0);
  
private:
  void build_grasp_msg(dual_manipulation_shared::grasp_trajectory& grasp_msg, const KDL::Frame& obj_ee_frame, int obj_id, std::string ee_frame_name);
  
  databaseWriter db_writer;
  databaseMapper db_mapper;
  
  uint yaw_steps_;
  uint end_effector_id_;
  std::string end_effector_frame_;
  double waypoint_height_;
};

#endif // TABLE_GRASP_MAKER_H
