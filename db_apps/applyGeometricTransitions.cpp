#include <ros/ros.h>
#include <ros/package.h>
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/databasewriter.h>
#include <ctime>

#include "grasp_creation_utilities/geometric_automatic_transitions.h"

// the sphere radius (default)/ box side (not implemented)  representing the hand workspace centered at
// (0.01, 0.0, 0.1) w.r.t. the palm_link frame
#define WS_SIZE 0.1

int main(int argc, char **argv)
{
  std::cout<<std::endl;
  std::cout<<"|Grasp db| -> apply geometric automatic transitions to db "<<std::endl;
  std::cout<<std::endl;

  ros::init(argc, argv, "apply_geometric_transmission");

  /* Assumptions
  -
  -
  -
  */

  GeometricAutomaticTransitions transitioner ( "full", WS_SIZE );

  std::string answer;
  std::cout << "are you ready? [y/n]" << std::endl << std::endl;
  std::cin >> answer;

  if( answer == "n" )
  {
    std::cout << "pussy" << std::endl;
    return -1;
  }

  if( answer == "y" )
  {
    std::cout << "good! grab a beer and relax while I do the job..." << std::endl;
  }
  else
  {
    std::cout << "Sorry, didn't understand what you wrote, let me know when you know how to read and type ;P" << std::endl;
    return -2;
  }

  sleep(3);

  if( !transitioner.writeTransitions() )
  {
    ROS_ERROR("Something happened during transition filter");
  }
  else
  {
    ROS_INFO("Finished the application of the geometric filter to prune all possible transitions!");
  }

  ros::spinOnce();

  return 0;

}
