#ifndef TABLE_GRASP_MAKER_H
#define TABLE_GRASP_MAKER_H

#include <string>
#include <kdl/frames.hpp>

#include "dual_manipulation_shared/databasemapper.h"
#include "dual_manipulation_shared/databasewriter.h"
#include "dual_manipulation_shared/grasp_trajectory.h"

// default values
#define END_EFFECTOR_ID 3		// the ID of the end-effector (table) to consider
#define END_EFFECTOR_FRAME "world"	// the frame to be used in the request
#define YAW_STEPS 8 			// how many steps to use when rotating the grasp
#define WAYPOINT_HEIGHT 0.1		// height of the waypoint used for pre-ungrasp and post-grasp

class tableGraspMaker
{
public:
  tableGraspMaker(std::string db_name = "test.db");
  
  bool read_data_from_file(std::string& obj_name, std::string& grasp_name, KDL::Frame& obj_ee_final, std::string filename = "/test/table_grasp_data.txt");
  bool create_table_grasps(int obj_id, std::string grasp_name, KDL::Frame obj_ee);
  
private:
  bool serialize_data(const dual_manipulation_shared::grasp_trajectory& grasp_msg, int object_id, int grasp_id);
  void build_grasp_msg(dual_manipulation_shared::grasp_trajectory& grasp_msg, const KDL::Frame& obj_ee_frame, int obj_id, std::string ee_frame_name);
  
  databaseWriter db_writer;
  databaseMapper db_mapper;
  
  uint yaw_steps_ = YAW_STEPS;
  uint end_effector_id_ = END_EFFECTOR_ID;
  std::string end_effector_frame_ = END_EFFECTOR_FRAME;
  double waypoint_height_ = WAYPOINT_HEIGHT;
  
};

#endif // TABLE_GRASP_MAKER_H
