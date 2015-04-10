#include "ros/ros.h"
#include <kdl/frames.hpp>
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/databasewriter.h>
#include "table_grasp_maker.h"
#include "specular_grasp_maker.h"
#include "named_automatic_transitions.h"

#define EE_ID 1
#define EE_FRAME "left_hand_palm_link"
#define JOINTS {"left_hand_synergy_joint"}
#define DB_NAME "containerB.db"

int main(int argc, char **argv)
{
  std::cout<<std::endl;
  std::cout<<"|Grasp db| -> create containerB db "<<std::endl;
  std::cout<<std::endl;

  ros::init(argc, argv, "create_containerB_db");

  /* Assumptions
  - workspaces, geometry, adjacencies, reachability
  - 1 object: containerB (with ID = 2)
  - 3 end effectors: left_hand right_hand table
  - 10 right hand grasps already serialized
  */
  
  // NOTE: STEP 1 > make specular grasps
  std::vector<uint> to_be_converted_grasp_ids = {1,2,3,4,5,6,7,8,9,10};
  // notice that names are equal, apart from the sign of y!!!
  std::vector<std::string> new_grasp_names = {"side_x+y+","handle_x-","side_x+y-","handle_x-_bottom","handle_rev_x-","handle_x+","side_x-y+","handle_rev_x-_replay","side_x+y-","side_x-y-_bottom"};
  bool top_bottom = false;

  assert(to_be_converted_grasp_ids.size()==new_grasp_names.size());
  
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
      ROS_FATAL_STREAM("Stopped transformation at grasp " << grasp_name << " > " << new_grasp_name << "!!!");
      return -1;
    }
  }
  
  // NOTE: STEP 2 > make table grasps
  tableGraspMaker table_grasps(db_name);
  KDL::Frame obj_ee(KDL::Vector(0.0,0.0,0.0));
  
  if(!table_grasps.create_table_grasps(2,"bottom",obj_ee))
  {
    ROS_FATAL_STREAM("Unable to create table grasps!!!");
    return -1;
  }
  
  // NOTE: STEP 3 > make transitions
  std::vector<std::string> prefixes({"bottom","handle_x-","side_x+y-","side_x-y-"});
  std::map<std::string,std::vector<std::string>> correspondences;
  correspondences["bottom"] = {"side","handle"};
  correspondences["handle_x-"] = {"handle_x+"};
  correspondences["side_x+y-"] = {"side_x-y+"};
  correspondences["side_x-y-"] = {"side_x+y+"};
  
  namedAutomaticTransitions nat(prefixes,correspondences,DB_NAME);
  
  bool write_ok = nat.write_transitions();
  if(!write_ok)
  {
    ROS_FATAL_STREAM("Unable to write transitions!!!");
    return -1;
  }

  ros::spinOnce();

  return 0;
}