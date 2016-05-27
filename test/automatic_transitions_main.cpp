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