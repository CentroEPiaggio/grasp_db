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

#include <iostream>
#include "grasp_creation_utilities/specular_grasp_maker.h"

#define EE_ID 1
#define EE_FRAME "left_hand_palm_link"
#define JOINTS {"left_hand_synergy_joint"}
#define DB_NAME "containerB.db"

int main(int argc, char **argv)
{
  std::cout<<std::endl;
  std::cout<<"|Dual manipulation| -> specular_grasp_maker "<<std::endl;
  std::cout<<std::endl;

  ros::init(argc, argv, "specular_grasp_maker");

  std::vector<std::string> grasp_links;

  bool ok = true;
  std::vector<uint> to_be_converted_grasp_ids = {1,2};
  std::vector<std::string> new_grasp_names = {"prova_scrittura1","prova_scrittura2"};
  std::vector<bool> top_bottoms = {false,false};

  assert(to_be_converted_grasp_ids.size()==new_grasp_names.size());
  assert(to_be_converted_grasp_ids.size()==top_bottoms.size());
  
  uint new_ee_id = EE_ID;
  std::string new_link_name = EE_FRAME;
  std::vector<std::string> new_joint_names = JOINTS;
  std::string db_name = DB_NAME;
  
  databaseMapper db_mapper(db_name);
  
  specularGraspMaker sgm(new_ee_id,new_link_name,new_joint_names,db_name);
  
  for(int i=0; i < to_be_converted_grasp_ids.size(); i++)
  {
    uint grasp_id = to_be_converted_grasp_ids.at(i);
    std::string new_grasp_name = new_grasp_names.at(i);
    bool top_bottom = top_bottoms.at(i);
    
    uint obj_id;
    uint ee_id;
    std::string grasp_name;

    obj_id = std::get<0>(db_mapper.Grasps.at(grasp_id));
    ee_id = std::get<1>(db_mapper.Grasps.at(grasp_id));
    grasp_name = std::get<2>(db_mapper.Grasps.at(grasp_id));
    
    ROS_INFO_STREAM("Converting grasp " << grasp_name << " > " << new_grasp_name << " (" << (i+1) << " out of " << to_be_converted_grasp_ids.size() << ")");

    bool transform_ok;
    transform_ok = sgm.transform_grasp(obj_id,grasp_id,new_grasp_name,top_bottom);
    
    if(!transform_ok)
    {
      ROS_ERROR_STREAM("Stopped transformation at grasp " << grasp_name << " > " << new_grasp_name);
      return -1;
    }
  }
  
  return 0;
}