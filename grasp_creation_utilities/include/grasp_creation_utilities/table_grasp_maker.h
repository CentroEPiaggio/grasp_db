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
#define YAW_STEPS 16			// how many steps to use when rotating the grasp
#define WAYPOINT_HEIGHT 0.1		// height of the waypoint used for pre-ungrasp and post-grasp

class tableGraspMaker
{
public:
  
  /**
   * @brief Constructor, uses default values if other values are not provided
   * 
   * @param db_name name of the database to use
   * @param yaw_steps how many times to replicate the grasp, rotating around global z-axis
   * @param ee_id the id of the end-effector (table) to use for saving the grasps
   * @param wp_height height of the wp over the table (for the table pre-grasp)
   * @param ee_frame (if different from world) the frame to write in the attObject frame field of the grasp: says to which frame the object will be attached, depends on the end-effector
   */
  tableGraspMaker(std::string db_name = "test.db", uint yaw_steps = YAW_STEPS, uint ee_id = END_EFFECTOR_ID, double wp_height = WAYPOINT_HEIGHT, std::string ee_frame = END_EFFECTOR_FRAME);
  
  bool read_data_from_file(std::string& obj_name, std::string& grasp_name, KDL::Frame& obj_ee_final, std::string filename = "/test/table_grasp_data.txt");
  bool create_table_grasps(int obj_id, std::string grasp_name, KDL::Frame obj_ee, uint64_t new_grasp_id=0);
  
private:
  bool serialize_data(const dual_manipulation_shared::grasp_trajectory& grasp_msg, int object_id, int grasp_id);
  void build_grasp_msg(dual_manipulation_shared::grasp_trajectory& grasp_msg, const KDL::Frame& obj_ee_frame, int obj_id, std::string ee_frame_name);
  
  databaseWriter db_writer;
  databaseMapper db_mapper;
  
  uint yaw_steps_;
  uint end_effector_id_;
  std::string end_effector_frame_;
  double waypoint_height_;
};

#endif // TABLE_GRASP_MAKER_H
