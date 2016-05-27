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
#include "grasp_creation_utilities/specular_grasp_maker.h"
#include "grasp_creation_utilities/named_automatic_transitions.h"

#define EE_ID 1

#define DB_NAME "containerB.db"
#define OBJECT_ID 2

// BEFORE SPECULARIZE; VALIDATE GRASPS
// TO VALIDATE GRASPS; GENERATE TABLE GRASPS AND TRANSITIONS
// AFTER TEST; SPECULARIZE GRASPS AND TRANSITIONS
#define SPECULARIZE false

// ONLY USFUL FOR SPECULARIZATION
#define EE_FRAME "left_hand_palm_link"
#define JOINTS {"left_hand_synergy_joint"}

int main(int argc, char **argv)
{
  std::cout<<std::endl;
  std::cout<<"|Grasp db| -> create containerB db "<<std::endl;
  std::cout<<std::endl;

  ros::init(argc, argv, "create_containerB_db");

  /* Assumptions
  - workspaces, geometry, adjacencies, reachability
  - 1 object: containerB (with ID = 2)
  - 3 end effectors: 1 left_hand 2 right_hand 3 table
  - 10 right hand grasps already serialized
  */

  uint new_ee_id = EE_ID;
  std::string new_link_name = EE_FRAME;
  std::vector<std::string> new_joint_names = JOINTS;
  std::string db_name = DB_NAME;
  
  // JUST CREATE SPECULAR GRASPS
  if(SPECULARIZE)
  {
    databaseMapper db_mapper( db_name );
  
    // NOTE: STEP 1 > make specular grasps for left hand
    std::vector<uint> specularized_grasps = {1,2,3,4,5,6,7};
    // notice that names are equal, apart from the sign of y!!!
    std::vector<std::string> specularized_grasp_names = {"side_x+y+_top","handle_x-y_top","side_x+y-_top","handle_x-y_bottom","handle_rev_x-y_top","handle_x+y_top","side_x-y+_top"};
    bool top_bottom = false;

    assert( specularized_grasps.size() == specularized_grasp_names.size() );

    specularGraspMaker sgm( new_ee_id, new_link_name, new_joint_names, db_name );
    
    for(int i=0; i < specularized_grasps.size(); i++)
    {
      uint grasp_id = specularized_grasps.at(i);
      std::string new_grasp_name = specularized_grasp_names.at(i);
      
      uint obj_id;
      uint ee_id;
      std::string grasp_name;

      obj_id = std::get<0>(db_mapper.Grasps.at(grasp_id));
      ee_id = std::get<1>(db_mapper.Grasps.at(grasp_id));
      grasp_name = std::get<2>(db_mapper.Grasps.at(grasp_id));
      
      ROS_INFO_STREAM("Converting grasp " << grasp_name << " > " << new_grasp_name << " (" << (i+1) << " out of " << specularized_grasps.size() << ")");

      bool transform_ok;
      transform_ok = sgm.transform_grasp( obj_id, grasp_id, new_grasp_name, top_bottom );
      
      if(!transform_ok)
      {
        ROS_FATAL_STREAM("Stopped transformation at grasp " << grasp_name << " > " << new_grasp_name << "!!!");
        return -1;
      }
    }

    // NOTE: STEP 2 > make transitions for specular grasps
    /* THIS IS NOW DONE WITH THE GEOMETRIC FILTER
    std::vector<std::string> prefixes({"bottom","handle_x-","side_x+y-","side_x-y-"});
    std::map<std::string,std::vector<std::string>> correspondences;
    correspondences["bottom"] = {"side","handle"};
    correspondences["handle_x-"] = {"handle_x+"};
    correspondences["side_x+y-"] = {"side_x-y+"};
    correspondences["side_x-y-"] = {"side_x+y+"};

    namedAutomaticTransitions nat( prefixes, correspondences, DB_NAME );

    bool write_ok = nat.write_transitions();
    if(!write_ok)
    {
      ROS_FATAL_STREAM("Unable to write transitions!!!");
      return -1;
    }
    */
  }
  
  // JUST CREATE TABLE GRASPS
  if(!SPECULARIZE)
  {
    // NOTE: STEP 1 > make table grasps
    tableGraspMaker table_grasps(db_name);
    KDL::Frame obj_ee(KDL::Vector(0.0,0.0,0.0));
    
    if(!table_grasps.create_table_grasps(OBJECT_ID, "bottom", obj_ee))
    {
      ROS_FATAL_STREAM("Unable to create table grasps!!!");
      return -1;
    }

    // NOTE: STEP 2 > make transitions for table grasps
    /* THIS IS NOW DONE WITH THE GEOMETRIC FILTER
    std::vector<std::string> prefixes({"bottom"});
    std::map<std::string,std::vector<std::string>> correspondences;
    correspondences["bottom"] = {"side","handle"};

    namedAutomaticTransitions nat( prefixes, correspondences, DB_NAME );

    bool write_ok = nat.write_transitions();
    if(!write_ok)
    {
      ROS_FATAL_STREAM("Unable to write transitions!!!");
      return -1;
    }
    */
  }

  // ros::spinOnce();

  return 0;
}
