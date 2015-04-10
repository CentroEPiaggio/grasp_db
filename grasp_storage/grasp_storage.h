#ifndef GRASP_STORAGE_H
#define GRASP_STORAGE_H

#include <iostream>
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <string>
#include <thread>
#include "ros/package.h"
#include "dual_manipulation_shared/databasemapper.h"
#include "dual_manipulation_shared/databasewriter.h"

#define WORLD_TF "/camera_link"
#define HAND_TF "/hand_palm_link"
#define OBJ_TF "/object"

class grasp_storage
{
public:
  grasp_storage(std::string db_name = "test.db");
  
  bool set_grasp_info(int object_id_, int end_effector_id_, std::string grasp_name_, std::vector< ros::Time > snapshots_, std::string world_tf_ = WORLD_TF, std::string hand_tf_ = HAND_TF, std::string object_tf_ = OBJ_TF);
  
  bool record_grasp();
  
  bool finished() {return stop_thread;};
  
  bool save_recording();
  
  void stop_recording();
  
  ~grasp_storage();
  
private:
  int object_id;
  int end_effector_id;
  std::string grasp_name;
  std::vector<ros::Time> snapshots;
  bool grasp_info_set = false;
  bool stop_thread = true;
  bool save_start = true;
  
  //     ros::NodeHandle node;
  tf::TransformListener tf;
  
  std::vector<geometry_msgs::Pose> obj0_trajectory;
  geometry_msgs::Pose hand_objF;
  tf::StampedTransform world_T_obj0;
  
  std::string world_tf, hand_tf, object_tf;
  boost::shared_ptr<std::thread> tf_periodic_listener;
  
  boost::shared_ptr<databaseMapper> db_mapper_;
  boost::shared_ptr<databaseWriter> db_writer_;
  
  bool get_transform(tf::StampedTransform& result, std::string target, std::string source, double wait_time = 1.0);
  void thread_body();
  
  bool save_start_pose();
  void record_trajectory_pose();
  void stop_record_trajectory_pose();
  void save_end_pose();
  
  bool serialize_data(uint new_grasp_id);
  int save_in_db(int obj_id, int ee_id, std::string name);
  void single_step(bool force_snapshot);
  
  void reset_grasp_storage();
};

#endif //GRASP_STORAGE_H