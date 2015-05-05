#ifndef GRASPS_SERIALIZER_H
#define GRASPS_SERIALIZER_H

#include <iostream>
#include <string>
#include <thread>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "dual_manipulation_shared/databasemapper.h"
#include "dual_manipulation_shared/databasewriter.h"

class GraspsSerializer
{
public:
  GraspsSerializer(){};

  bool parseGraspInfo(XmlRpc::XmlRpcValue grasps);
    
  bool updateDatabase();
 
  ~GraspsSerializer() {};
  
private:

  boost::shared_ptr<databaseMapper> db_mapper_;
  boost::shared_ptr<databaseWriter> db_writer_;

  // grasp info
  int object_id_;
  std::string object_name_, dataset_;
  struct rec_grasp
  {
    int end_effector_id;
    std::string grasp_name;
    std::vector<ros::Time> waypoints;
    ros::Time post_grasp;
    std::vector<geometry_msgs::Pose> waypoints_obj_T_hand;
    geometry_msgs::Pose post_hand_T_obj;
  };
  std::vector<rec_grasp> rec_grasps_;
  std::string world_tf_, hand_tf_, object_tf_;

  // control parameters
  bool grasp_info_set_ = false;

  // holder of the whole tf tree
  tf::Transformer transformer_;
  
  void openDB(std::string db_name);
  bool serializeGrasp(uint grasp_id, const rec_grasp grasp);
};

#endif //GRASPS_SERIALIZER_H
