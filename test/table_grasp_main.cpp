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
#include <string>
#include "dual_manipulation_shared/databasemapper.h"
#include "grasp_creation_utilities/table_grasp_maker.h"

int main()
{
  std::string db_name = "test.db";
  tableGraspMaker tGraspMaker(db_name);
  databaseMapper db_mapper(db_name);
  
  std::string obj_name;
  std::string grasp_name;
  KDL::Frame obj_ee_final;
  
  // read grasp data from file
  if(!tGraspMaker.read_data_from_file(obj_name, grasp_name, obj_ee_final,"/test/table_grasp_data.txt"))
  {
    ROS_ERROR_STREAM("Unable to read grasp file - returning");
    return -1;
  }
  
  int obj_id = -1;
  
  for(auto& obj:db_mapper.Objects)
  if(obj.second.name == obj_name)
  {
    obj_id = obj.first;
    break;
  }
  
  if(obj_id == -1)
  {
    ROS_ERROR_STREAM("Object " << obj_name << " is not present in the DB - returning");
    return -1;
  }
  
  if(!tGraspMaker.create_table_grasps(obj_id, grasp_name, obj_ee_final))
  {
    ROS_ERROR_STREAM("Object " << obj_name << " is not present in the DB - returning");
    return -1;
  }
  
  ROS_INFO_STREAM("Everything done! Change grasp file and restart!");
  return 0;
}