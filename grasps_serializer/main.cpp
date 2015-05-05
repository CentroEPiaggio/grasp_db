#include <fstream>

#include <ros/ros.h>

#include "grasps_serializer.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp serialization");

  ros::NodeHandle node;
  
  GraspsSerializer serializer;
  
  // read ros parameter defined by the yaml file within the node namespace
  XmlRpc::XmlRpcValue grasps;
  node.getParam( node.getNamespace(), grasps );

  // parse the info
  if( !serializer.parseGraspInfo( grasps ) )
  {
    ROS_ERROR_STREAM("Unable to parse required information in GraspsSerializer - returning...");
    return -1;
  }

  // and update the database
  if( !serializer.updateDatabase() )
  {
    ROS_ERROR_STREAM("Something went wrong with the update, this program didn't finished properly");
    return -1;
  }

  // done
  ros::spinOnce();

  return 0;
}