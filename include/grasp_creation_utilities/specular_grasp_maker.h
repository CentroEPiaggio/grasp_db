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

#ifndef SPECULAR_GRASP_MAKER_H
#define SPECULAR_GRASP_MAKER_H

#include <string>
#include <kdl/frames.hpp>

#include "dual_manipulation_shared/databasemapper.h"
#include "dual_manipulation_shared/databasewriter.h"
#include "dual_manipulation_shared/grasp_trajectory.h"

class specularGraspMaker
{
public:
  
  /**
   * @brief Constructor, needs all values to be passed in order to work properly (test.db used by default)
   * 
   * @param ee_id the id of the end-effector to use for saving the grasps
   * @param ee_frame the frame to write in the attObject frame field of the grasp: says to which frame the object will be attached, depends on the end-effector
   * @param ee_joint_names vector of joint names to be written in the new grasp
   * @param db_name name of the database to use
   * 
   */
  specularGraspMaker(uint ee_id, std::string ee_frame, const std::vector< std::string >& new_joint_names, std::string db_name = "test.db");
  
  ~specularGraspMaker(){};

  /**
   * @brief transform a grasp given in input its @p obj_id and @p grasp_id into a new grasp, automatically finding the new grasp_id from the database
   * This function transform the grasp specified by @p obj_id and @p grasp_id into a new grasp for the end-effector specified in the constructor. The operations are: read the original grasp from file, create a new entry in the database, transform the grasp using specularity, and write the results in a new grasp file
   * 
   * @param obj_id the object of the grasp
   * @param grasp_id the id of the grasp
   * @param new_grasp_name name to give to the new grasp
   * @param top_bottom flag specifying if we want to transform the grasp in a top-bottom fashion (top grasp becoming a bottom grasp, sideA-high becoming a sideA-low) instead of just having a new grasp which is specular w.r.t. XZ plane
   * @param new_grasp_id id of the new grasp to write (autochosen if left empty)
   * @param ec_id environment constraint id - zero by default (meaningless - could not be used with new transition types)
   * 
   * @return true on success
   */
  bool transform_grasp(uint obj_id, uint grasp_id, std::string new_grasp_name, bool top_bottom, uint new_grasp_id = 0, constraint_id ec_id = 0);
  
private:
  
  void transform_specular_y(KDL::Frame& pose);
  void transform_specular_y(std::vector< KDL::Frame >& poses);
  void transform_premultiply(std::vector< KDL::Frame >& poses, KDL::Frame frame);
  
  /**
   * Given a grasp as input, performs the necessary operations on the same message to obtain a new, specular grasp. Specularity is w.r.t. XZ plane or top-bottom depending on @p top_bottom flag (always transforming between right and left handed grasps)
   */
  void transform_grasp_specularity(dual_manipulation_shared::grasp_trajectory& grasp_msg, /*uint new_grasp_id, */uint obj_id, std::string new_link_name, const std::vector< std::string >& new_joint_names, bool top_bottom);

  std::string db_name_;
  uint end_effector_id_;
  std::string end_effector_frame_;
  std::vector<std::string> joint_names_;
};

#endif // SPECULAR_GRASP_MAKER_H
