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

#ifndef NAMED_AUTOMATIC_TRANSITIONS_H
#define NAMED_AUTOMATIC_TRANSITIONS_H

#include <ros/ros.h>
#include <string>
#include <XmlRpcValue.h>

#include "dual_manipulation_shared/databasemapper.h"
#include "dual_manipulation_shared/databasewriter.h"
#include "dual_manipulation_shared/parsing_utils.h"

class namedAutomaticTransitions
{
public:
  
  /**
   * @brief Default constructor, looks for parameters in the parameterServer
   */
  namedAutomaticTransitions(std::string db_name = "test.db");
  
  /**
   * @brief Constructor, needs all values to be passed in order to work properly (test.db used by default)
   * 
   * @param prefixes vector of all grasp_name prefixed to consider
   * @param correspondences map containing entries for each element of @p prefixes, a vector of other prefixes to consider a transition good (for different end-effectors)
   * @param db_name name of the database to use
   */
  namedAutomaticTransitions(const std::vector< std::string >& prefixes, const std::map<std::string,std::vector<std::string>>& correspondences, std::string db_name = "test.db");
  
  ~namedAutomaticTransitions(){};

  /**
   * @brief writes the resulting transitions to the grasp DB
   * 
   * @return true on success
   */
  bool write_transitions();
  
private:
  void parseParameters(XmlRpc::XmlRpcValue& params);
  
  ros::NodeHandle* node;
  
  boost::shared_ptr<databaseMapper> db_mapper_;
  boost::shared_ptr<databaseWriter> db_writer_;

  std::string db_name_;
  std::vector<std::string> prefixes_;
  std::map<std::string,std::vector<std::string>> correspondences_;
  
  XmlRpc::XmlRpcValue params;
};

#endif // NAMED_AUTOMATIC_TRANSITIONS_H
