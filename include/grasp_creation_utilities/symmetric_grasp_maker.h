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

#ifndef SYMMETRIC_GRASP_MAKER_H
#define SYMMETRIC_GRASP_MAKER_H

#include <string>
#include <kdl/frames.hpp>

#include "dual_manipulation_shared/databasemapper.h"
#include "dual_manipulation_shared/databasewriter.h"
#include "dual_manipulation_shared/grasp_trajectory.h"

class symmetricGraspMaker
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
    symmetricGraspMaker(uint ee_id, std::string ee_frame, const std::vector< std::string >& new_joint_names, std::string db_name = "test.db");
    
    ~symmetricGraspMaker(){};
    
    /**
     * @brief transform a grasp given in input its @p obj_id and @p grasp_id into a new set of grasps, automatically finding the new grasp_ids from the database or using the vector provided as input
     * This function transform the grasp specified by @p obj_id and @p grasp_id into a new set of grasps for the end-effector specified in the constructor. The operations are: read the original grasp from file, then for each of the new grasps to create:
     * - create a new entry in the database
     * - transform the grasp using rotational symmetry around the rot_axis given as input applyed to the object (pre-multiplication of all matrixes)
     * - write the results in a new grasp file
     * This procedure rolls back if any of the above steps doesn't work, to the point where the last cycle worked in full.
     * 
     * @param obj_id the object of the grasp
     * @param grasp_id the id of the grasp
     * @param new_grasp_name name to give to the new grasp
     * @param new_grasp_id vector of id of the new grasps to write (autochosen if left empty)
     * @param how_many_rot number of total rotations (1 already present - so 1 less will be generated)
     * @param ec_id environment constraint id - zero by default (meaningless - could not be used with new transition types)
     * 
     * @return true on success
     */
    bool transform_grasp(uint obj_id, uint grasp_id, std::string new_grasp_base_name, std::vector< uint > new_grasp_ids, uint how_many_rot, KDL::Vector rot_axis, KDL::Frame rotFrame_obj, constraint_id ec_id = 0);
    
private:
    
    void transform_premultiply(std::vector< KDL::Frame >& poses, KDL::Frame frame);
    
    /**
     * Given a grasp as input, performs the necessary operations on the same message to obtain a new, symmetric grasp w.r.t. a rotation around the axis applied to the rotFrame_obj frame, both specified in the transform_grasp function.
     */
    void transform_grasp_symmetry(dual_manipulation_shared::grasp_trajectory& grasp_msg, uint obj_id, std::string new_link_name, const std::vector< std::string >& new_joint_names, KDL::Frame rotFrame_obj);
    
    std::string db_name_;
    uint end_effector_id_;
    std::string end_effector_frame_;
    std::vector<std::string> joint_names_;
};

#endif // SYMMETRIC_GRASP_MAKER_H
