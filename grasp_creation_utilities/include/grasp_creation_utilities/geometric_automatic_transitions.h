#ifndef GEOMETRIC_AUTOMATIC_TRANSITIONS_H
#define GEOMETRIC_AUTOMATIC_TRANSITIONS_H

#include <ros/ros.h>
#include <string>

#include "dual_manipulation_shared/databasemapper.h"
#include "dual_manipulation_shared/databasewriter.h"
#include "dual_manipulation_shared/parsing_utils.h"

// generated from message
#include "dual_manipulation_shared/grasp_trajectory.h"

class GeometricAutomaticTransitions
{
public:
  
  /**
   * @brief Default constructor, looks for parameters in the parameterServer
   */
  GeometricAutomaticTransitions(std::string db_name = "full.db", double ws_hand_size = 0.1);
  
  ~GeometricAutomaticTransitions(){};

  /**
   * @brief writes the resulting transitions to the grasp DB
   * 
   * @return true on success
   */
  bool writeTransitions();
  
private:
  
  boost::shared_ptr<databaseMapper> db_mapper_;
  boost::shared_ptr<databaseWriter> db_writer_;

  std::string db_name_;
  double hand_ws_size_;

  // check the grasp of a grasp_msg with the trajectories of the other grasp_ms
  bool isHandTableGraspTransitionValid(const KDL::Frame palm_inObject);

  bool isGraspTransitionValid(const dual_manipulation_shared::grasp_trajectory& G1, const int ee1_mobility,
                              const dual_manipulation_shared::grasp_trajectory& G2, const int ee2_mobility);

  // single condition to be a valid configuration, a simple collision check
  bool isHandTableConfigValid(const KDL::Frame palm_inObject);

  bool isHandHandConfigValid(const KDL::Frame palm2_inPalm1);

  // deserialize a grasp. this function can be avoided, it only calls the deserialize IK.
  bool readGrasp(const uint obj_id, const uint grasp_id, dual_manipulation_shared::grasp_trajectory& grasp_msg);

};

#endif // GEOMETRIC_AUTOMATIC_TRANSITIONS_H
