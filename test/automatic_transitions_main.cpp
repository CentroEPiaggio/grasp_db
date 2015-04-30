#include <iostream>
#include "grasp_creation_utilities/named_automatic_transitions.h"

#define DB_NAME "top-top.db"

int main(int argc, char **argv)
{
  std::cout<<std::endl;
  std::cout<<"|Dual manipulation| -> specular_grasp_maker "<<std::endl;
  std::cout<<std::endl;

  ros::init(argc, argv, "specular_grasp_maker");

  std::vector<std::string> prefixes({"top_","top"});
  std::map<std::string,std::vector<std::string>> correspondences;
  correspondences[prefixes.at(0)] = {"Bottom_1","Bottom_2"};
  correspondences[prefixes.at(1)] = {"Bottom_2"};
  
  namedAutomaticTransitions nat(prefixes,correspondences,DB_NAME);
  
  bool write_ok = nat.write_transitions();
  if(!write_ok)
  {
    ROS_ERROR_STREAM("Unable to write transitions!!!");
    return -1;
  }
  
  return 0;
}