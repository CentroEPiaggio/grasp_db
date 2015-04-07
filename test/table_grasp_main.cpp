#include <ros/ros.h>
#include <string>
#include "dual_manipulation_shared/databasemapper.h"
#include "table_grasp_maker.h"

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
  if(std::get<0>(obj.second) == obj_name)
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