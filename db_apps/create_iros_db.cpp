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

#include "ros/ros.h"
#include <kdl/frames.hpp>
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/databasewriter.h>
#include "grasp_creation_utilities/table_grasp_maker.h"

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Grasp db| -> create iros db "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "create_iros_db");

    /* Assumptions
    - workspaces, geometry, adjacencies, reachability
    - 1 object: Cylinder
    - 3 end effectors: table leftarm rightarm
    - hand grasps alreay serialized
    - selected hand grasps in the database (TODO write from here)
      currently the hand grasps are one for each hand!!!!!
    */
    
    tableGraspMaker table_grasps("top-top.db");
    KDL::Frame obj_ee(KDL::Vector(0.0,0.0,-0.1));
    table_grasps.create_table_grasps(1,"Bottom",obj_ee);
    
    databaseMapper mapper("top-top.db");
    endeffector_id left,right;
    for (auto ee : mapper.EndEffectors)
    {
        if (std::get<0>(ee.second)=="left_hand")
            left=ee.first;
        if (std::get<0>(ee.second)=="right_hand")
            right=ee.first;
    }
    grasp_id left_top_grasp, right_top_grasp;
    for (auto grasp:mapper.Grasps)
    {
        if (std::get<1>(grasp.second)==left)
            left_top_grasp=grasp.first;
        if (std::get<1>(grasp.second)==right)
            right_top_grasp=grasp.first;
    }
    databaseWriter writer("top-top.db");
    
    for (auto grasp:mapper.Grasps)
    {
        if (grasp.first!=left_top_grasp && grasp.first!=right_top_grasp)
        {
            writer.writeNewTransition(left_top_grasp,grasp.first);
            writer.writeNewTransition(right_top_grasp,grasp.first);
        }
    }
    ros::spinOnce();

    return 0;
}