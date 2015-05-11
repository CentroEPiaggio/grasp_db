#include "grasp_creation_utilities/geometric_automatic_transitions.h"
#include "dual_manipulation_shared/serialization_utils.h"

#include <kdl_conversions/kdl_msg.h>
#include <string>

// AWARE THAT THIS VALUE MUST CORRESPOND WITH THAT USED IN THE CREATE_FULL_DB.CPP
#define OBJ_GRASP_FACTOR 1000

// PUBLIC

GeometricAutomaticTransitions::GeometricAutomaticTransitions( std::string db_name, double hand_ws_size):db_name_(db_name), hand_ws_size_(hand_ws_size)
{ 
  db_mapper_ = boost::shared_ptr<databaseMapper>(new databaseMapper(db_name_));
  db_writer_ = boost::shared_ptr<databaseWriter>(new databaseWriter(db_name_));
  return;
}

bool GeometricAutomaticTransitions::writeTransitions()
{
  // temporary vars to store each grasp and relative transform to check
  dual_manipulation_shared::grasp_trajectory G1;
  dual_manipulation_shared::grasp_trajectory G2;

  // grasp1 is the reference grasp posture
  // grasp2, with all waypoints and post grasp is checked agains grasp1
  // transitions are stored grasp1 -> grasp2 (note that grasp2 -> grasp1 might not be valid)

  // traverse all grasps with all grasps of all end-effector per object
  for( auto grasp1:db_mapper_->Grasps )
  {   
    for( auto grasp2:db_mapper_->Grasps )
    {
      // do not waste time with the same grasp
      if( grasp1.first == grasp2.first )
      {
        std::cout << "Same grasp, discarded: " << grasp1.first << " " << grasp1.first << std::endl;
        continue;
      }

      uint object1d = std::get<0>(grasp1.second);
      uint object2d = std::get<0>(grasp2.second);
      if( object1d == object2d )
      {
        uint ee1 = std::get<1>(grasp1.second);
        uint ee2 = std::get<1>(grasp2.second);

        // skip transition between same ee, future work !
        if( ee1 == ee2 )
        {
          std::cout << "Same ee, discarded: " << ee1 << " " << ee2 << std::endl;
          continue;
        }

        int is_ee1_movable = std::get<1>( db_mapper_->EndEffectors.at( ee1 ));
        // std::cout << "ee1 " << ee1 << " is_ee1_movable " << is_ee1_movable << std::endl;
        int is_ee2_movable = std::get<1>( db_mapper_->EndEffectors.at( ee2 ));
        // std::cout << "ee2 " << ee2 << " is_ee2_movable " << is_ee2_movable << std::endl;

        // extract grasp 1, use this as the reference grasp posture
        if( !readGrasp( object1d, grasp1.first, G1) )
        {
          std::cout << "Couldn't read grasp 1, check database." << std::endl;
          return false;
        }

        // extract grasp 2, to be checked against grasp 1
        if( !readGrasp( object1d, grasp2.first, G2) )
        {
          std::cout << "Couldn't read grasp 2, check database." << std::endl;
          return false;
        }
          
        if( !(isGraspTransitionValid( G1, is_ee1_movable, G2, is_ee2_movable )) )
        {
          std::cout << "Filter discarded transition in object #" << object1d << ", grasp " << grasp1.first << " by ee " << ee1 << " -> " << " grasp " << grasp2.first << " by ee " << ee2 << std::endl;
        }
        else
        {
          std::cout << "Adding transition in object #" << object1d << ", grasp " << grasp1.first << " by ee " << ee1 << " -> " << " grasp " << grasp2.first << " by ee " << ee2 << std::endl;
          db_writer_->writeNewTransition( grasp1.first, grasp2.first );
        }
      }
    }
  }
  return true;
}

// PRIVATE

bool GeometricAutomaticTransitions::readGrasp(const uint obj_id, const uint grasp_id, dual_manipulation_shared::grasp_trajectory& grasp_msg)
{
  uint id = grasp_id % OBJ_GRASP_FACTOR;
  if( !deserialize_ik( grasp_msg, "object" + std::to_string(obj_id) + "/grasp" + std::to_string(id) ) )
  {
    std::cout << "Error in deserialization object" + std::to_string(obj_id) + "/grasp" + std::to_string(id) << "!" << std::endl;
    return false;
  }
  return true;
}

// private

bool GeometricAutomaticTransitions::isGraspTransitionValid(const dual_manipulation_shared::grasp_trajectory& G1, const int ee1_mobility,
                                                           const dual_manipulation_shared::grasp_trajectory& G2, const int ee2_mobility)
{
  // temp var
  KDL::Frame G_kdl;

  // get the post-grasp posture of grasp 1
  KDL::Frame G1_kdl;
  tf::poseMsgToKDL( G1.attObject.object.mesh_poses.front(), G1_kdl );

  // check grasp 1 with all waypoints of grasp 2
  for( auto waypoint:G2.ee_pose )
  {
    //get the current waypoint posture of grasp 2
    tf::poseMsgToKDL( waypoint, G_kdl);

    // check if the current combination is valid
    //std::cout << "checking waypoint.." << std::endl;

    // select the config condition according to mobility
    if( ee1_mobility == 1 && ee2_mobility == 1)
    {
      // std::cout << "Possible MOVABLE-MOVABLE transition in object #" << object1d << ", grasp " << grasp1.first << " by ee " << ee1 << " -> " << " grasp " << grasp2.first << " by ee " << ee2 << std::endl;
      
      if( !(isHandHandConfigValid( G1_kdl*G_kdl )) )
      {
        // only one that is not valid make the whole transition invalid
        // std::cout << "Post-grasp filter not passed"
        return false;
      }
    }
    if( ee1_mobility == 1 && ee2_mobility == 0 )
    {
      // std::cout << "Possible MOVABLE-FIX transition in object #" << object1d << ", grasp " << grasp1.first << " by ee " << ee1 << " -> " << " grasp " << grasp2.first << " by ee " << ee2 << std::endl;

      if( !(isHandTableConfigValid( (G1_kdl*G_kdl).Inverse() )) )
      {
        // only one that is not valid make the whole transition invalid
        // std::cout << "Post-grasp filter not passed"
        return false;
      }
    }
    if( ee1_mobility == 0 && ee2_mobility == 1 )
    {
        // std::cout << "Possible FIX-MOVABLE transition in object #" << object1d << ", grasp " << grasp1.first << " by ee " << ee1 << " -> " << " grasp " << grasp2.first << " by ee " << ee2 << std::endl;
        if( !(isHandTableConfigValid( G1_kdl*G_kdl )) )
        {
          // only one that is not valid make the whole transition invalid
          // std::cout << "Post-grasp filter not passed"
          return false;
        }
    }
    if( (ee1_mobility == 0 && ee2_mobility == 0) )
    {
      // no transition if ee is fixed
      return false;
    }
  }

  // if no one complained, make the transition valid
  return true;
}

// config conditions

bool GeometricAutomaticTransitions::isHandTableConfigValid(const KDL::Frame palm_inTable)
{
  // this are points expressed in the palm
  KDL::Vector p1_inPalm(0.01, 0.1, 0.1);
  KDL::Vector p2_inPalm(0.01, 0.0, 0.0);
  KDL::Vector p3_inPalm(0.01, -0.1, 0.1);

  KDL::Vector p1_inTable = palm_inTable*p1_inPalm;
  KDL::Vector p2_inTable = palm_inTable*p2_inPalm;
  KDL::Vector p3_inTable = palm_inTable*p3_inPalm;

  if( p1_inTable.z() > 0.0 && p2_inTable.z() > 0.0 && p3_inTable.z() > 0.0 )
  {
    return true;
  }
  else
  {
    std::cout << "One of the 3 points defining the hand ws collision geometry with the table is below the table" << std::endl;
    return false;
  }
}

bool GeometricAutomaticTransitions::isHandHandConfigValid(const KDL::Frame palm2_inPalm1)
{
  // sphere to sphere collision check boils down to check whether the origin of the relative frame is further than twice the radius of the sphere.
  KDL::Vector ws1_center_inPalm1 (0.01, 0.0, 0.1);
  KDL::Vector ws2_center_inPalm1 = palm2_inPalm1*ws1_center_inPalm1;

  KDL::Vector p = ws2_center_inPalm1 - ws1_center_inPalm1;

  if( p.Norm() < 2*hand_ws_size_ )
  {
    std::cout << "Distance between hand ws centers is " << p.Norm() << ", and should be h than 2*radius of sphere defining ws of the hand set to " << 2*hand_ws_size_ << std::endl;
    return false;
  }
  return true;
}
