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
#include "ros/package.h"
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/databasewriter.h>
#include <ctime>

#define OBJ_GRASP_FACTOR 1000

std::vector<std::string> object_list({"cylinder","containerB","mugD","bowlA","bowlB","containerA","kitchenUtensilA","panA","pot"});

int main(int argc, char **argv)
{
  std::cout<<std::endl;
  std::cout<<"|Grasp db| -> create full db "<<std::endl;
  std::cout<<std::endl;

  ros::init(argc, argv, "create_full_db");

  /* Assumptions
  - a file named basic.db exists with tables workspace(s), end-effector, adjacency, and reachability all filled
  - various different files of single object db's exist (cylinder.db, containerB.db, ...)
  - no object has more than 999 grasps
  */
  
  // get current date/time to use in the naming of the full DB
  time_t rawtime;
  struct tm * timeinfo;
  char buffer [15];
  std::time(&rawtime);
  timeinfo = std::localtime(&rawtime);
  std::strftime(buffer,15,"%Y%m%d_%H%M",timeinfo);
  // std::cout << buffer << std::endl;
  
  // copy empty.db to a new full_{NOW}.db
  std::string path = ros::package::getPath("dual_manipulation_grasp_db");
  std::string new_db_name("full_" + std::string(buffer,15));
  std::string command = "cp " + path + "/basic.db " + path + "/" + new_db_name + "\n";
  system(command.c_str());
  
  databaseWriter db_writer(new_db_name);
  
  for(auto object:object_list)
  {
    // open a single object database
    databaseMapper db_mapper(object + ".db");
    
    // write objects (IDs must stay the same to have access to grasps!!!)
    for(auto obj:db_mapper.Objects)
    {
      int obj_id(obj.first);
      std::string obj_name(std::get<0>(obj.second));
      std::string obj_path(std::get<1>(obj.second));
      KDL::Frame obj_center(std::get<2>(obj.second));
      db_writer.writeNewObject(obj_id,obj_name,obj_path,obj_center);
    }
    
    // write grasps changing graspID to 1000*objectID+graspID
    for(auto grasp:db_mapper.Grasps)
    {
      int grasp_id(grasp.first);
      int object_id(std::get<0>(grasp.second));
      int ee_id(std::get<1>(grasp.second));
      std::string name(std::get<2>(grasp.second));
      db_writer.writeNewGrasp(grasp_id+OBJ_GRASP_FACTOR*object_id,object_id,ee_id,name);
    }
    
    // write transitions, applying the same change of ID as above
    for(auto gt:db_mapper.Grasp_transitions)
    {
      int source_id(gt.first+OBJ_GRASP_FACTOR*std::get<0>(db_mapper.Grasps.at(gt.first)));
      for(auto target:gt.second)
      {
        int target_id(target+OBJ_GRASP_FACTOR*std::get<0>(db_mapper.Grasps.at(target)));
        db_writer.writeNewTransition(source_id,target_id);
      }
    }
  }

  // this is not working, so doing it manually
  // copy the full database with no timestamp
  // command = "cp " + path + "/" + new_db_name + " " + path + "/full";
  // std::cout << command << std::endl;
  // system(command.c_str());

  return 0;
}