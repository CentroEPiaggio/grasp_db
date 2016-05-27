/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Alessandro Settimi, Hamal Marino, Mirko Ferrati, Centro di Ricerca "E. Piaggio", University of Pisa
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

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
