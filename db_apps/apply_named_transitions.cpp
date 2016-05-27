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

#include "ros/ros.h"
#include <grasp_creation_utilities/named_automatic_transitions.h>
#include <XmlRpcValue.h>
#include <dual_manipulation_shared/parsing_utils.h>

// Make all transitions based on names

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Grasp db| -> apply named transitions to db "<<std::endl;
    std::cout<<std::endl;
    
    ros::init(argc, argv, "apply_named_transitions");
    ros::NodeHandle node;
    ros::AsyncSpinner aspin(1);
    aspin.start();
    XmlRpc::XmlRpcValue ant_params;
    
    if (!node.getParam("apply_named_transitions_parameters", ant_params))
    {
        ROS_FATAL_STREAM("Unable to parse parameters from server!!!");
        std::cout << "Unable to parse parameters from server!!!" << std::endl;
        return -1;
    }
    
    std::string db_name;
    std::vector<std::string> prefixes;
    std::map<std::string,std::vector<std::string>> correspondences;
    parseSingleParameter(ant_params,db_name,"db_name");
    parseSingleParameter(ant_params,prefixes,"prefixes");
    
    for(auto pref:prefixes)
    {
        correspondences[pref];
        parseSingleParameter(ant_params["correspondences"],correspondences[pref],pref);
    }

    namedAutomaticTransitions nat( prefixes, correspondences, db_name );
    
    std::cout << "db_name: " << db_name << std::endl << "prefixes:";
    for(auto p:prefixes)
    {
        std::cout << " " << p << ":";
        for(auto c:correspondences[p])
            std::cout << " " << c;
        std::cout << std::endl;
    }
    std::cout << std::endl;
    
    bool write_ok = nat.write_transitions();
    if(!write_ok)
    {
        ROS_FATAL_STREAM("Unable to write transitions!!!");
        return -1;
    }
    
    return 0;
}