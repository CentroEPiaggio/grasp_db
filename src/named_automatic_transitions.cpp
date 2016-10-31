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

#include "grasp_creation_utilities/named_automatic_transitions.h"
#include <string>

#define DEBUG 1

namedAutomaticTransitions::namedAutomaticTransitions(std::string db_name):db_name_(db_name)
{
    node = new ros::NodeHandle();
    if (node->getParam("dual_manipulation_grasp_db", params))
        parseParameters(params);
    
    db_mapper_ = boost::shared_ptr<databaseMapper>(new databaseMapper(db_name_));
    db_writer_ = boost::shared_ptr<databaseWriter>(new databaseWriter(db_name_));
}

namedAutomaticTransitions::namedAutomaticTransitions(const std::vector< std::string >& prefixes, const std::map< std::string, std::vector< std::string > >& correspondences, std::string db_name):prefixes_(prefixes),correspondences_(correspondences),db_name_(db_name)
{
    for(auto pref:prefixes_)
    {
        if(correspondences_.count(pref) == 0 || correspondences_.at(pref).empty())
            ROS_FATAL_STREAM("namedAutomaticTransitions : no correspondences found for " << pref << " - aborting!");
        
        assert(correspondences_.count(pref) != 0 && !correspondences_.at(pref).empty());
    }
    
    db_mapper_ = boost::shared_ptr<databaseMapper>(new databaseMapper(db_name_));
    db_writer_ = boost::shared_ptr<databaseWriter>(new databaseWriter(db_name_));
}

void namedAutomaticTransitions::parseParameters(XmlRpc::XmlRpcValue& params)
{
    ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    
    assert(params.hasMember("prefixes"));
    assert(params.hasMember("correspondences"));
    
    parseSingleParameter(params,prefixes_,"prefixes",1);
    
    std::map<std::string,std::vector<std::string>> corr_tmp;
    for(auto pref:prefixes_)
    {
        parseSingleParameter(params["correspondences"],corr_tmp[pref],pref,1);
        
        if(corr_tmp.at(pref).empty())
            ROS_FATAL_STREAM("namedAutomaticTransitions::parseParameters : no correspondences found for " << pref << ", check the yaml file - aborting!");
        assert(!corr_tmp.at(pref).empty());
    }
    correspondences_.swap(corr_tmp);
}

bool compare_and_store(const std::string& main_str,const std::string& sub_str,std::map<std::string,std::vector<uint>>& store_map,uint new_elem)
{
    if(main_str.compare(0,sub_str.length(),sub_str) != 0)
        return false;
    
    // found a match, check if I got the same alredy
    if(std::find(store_map[sub_str].begin(),store_map[sub_str].end(),new_elem) != store_map[sub_str].end())
        return false;
    
    // I dind't: store it!
    store_map[sub_str].push_back(new_elem);
    
    return true;
}

bool namedAutomaticTransitions::write_transitions()
{
    // map on grasp prefix all associated pairs <ee_id,grasp_id> (using two vectors for clarity
    std::map<std::string,std::vector<uint>> grasp_id_from_prefix;
    std::map<std::string,std::vector<uint>> ee_id_from_prefix;
    std::map<std::string,std::vector<uint>> obj_id_from_prefix;
    
    for(auto grasp:db_mapper_->Grasps)
    {
        uint grasp_id = grasp.first;
        
        uint obj_id = std::get<0>(grasp.second);
        uint ee_id = std::get<1>(grasp.second);
        std::string grasp_name = std::get<2>(grasp.second);
        
        for(auto pref:prefixes_)
        {
            if(compare_and_store(grasp_name,pref,grasp_id_from_prefix,grasp_id))
            {
                ee_id_from_prefix[pref].push_back(ee_id);
                obj_id_from_prefix[pref].push_back(obj_id);
            }
            
            // do the same for each correspondence
            for(auto corr:correspondences_.at(pref))
            {
                if(compare_and_store(grasp_name,corr,grasp_id_from_prefix,grasp_id))
                {
                    ee_id_from_prefix[corr].push_back(ee_id);
                    obj_id_from_prefix[corr].push_back(obj_id);
                }
            }
        }
    }
    
    #if DEBUG
    for(auto grasp:grasp_id_from_prefix)
    {
        std::cout << "grasp prefix " << grasp.first << " -> Grasp IDs: ";
        for(auto id:grasp.second)
            std::cout << id << ", ";
        std::cout << std::endl;
    }
    for(auto ee:ee_id_from_prefix)
    {
        std::cout << "end-effector prefix " << ee.first << " -> Ee IDs: ";
        for(auto id:ee.second)
            std::cout << id << ", ";
        std::cout << std::endl;
    }
    #endif
    db_writer_->open_global();
    for(auto pref:prefixes_)
    {
        for(auto corr:correspondences_.at(pref))
        {
            for(int i1=0; i1<ee_id_from_prefix.at(pref).size(); i1++)
            {
                uint pref_ee_id = ee_id_from_prefix.at(pref).at(i1);
                uint pref_obj_id = obj_id_from_prefix.at(pref).at(i1);
                
                for(int i2=0; i2<ee_id_from_prefix.at(corr).size(); i2++)
                {
                    uint corr_ee_id = ee_id_from_prefix.at(corr).at(i2);
                    uint corr_obj_id = obj_id_from_prefix.at(corr).at(i2);
                    
                    if(pref_obj_id == corr_obj_id && pref_ee_id != corr_ee_id)
                    {
                        uint pref_grasp_id = grasp_id_from_prefix.at(pref).at(i1);
                        uint corr_grasp_id = grasp_id_from_prefix.at(corr).at(i2);
                        
                        int write_res = db_writer_->writeNewTransition(pref_grasp_id,corr_grasp_id,true);
                        //      int write_res2 = db_writer_->writeNewTransition(corr_grasp_id,pref_grasp_id);
                        int write_res2 = write_res;
                        
                        std::string pref_to_corr(std::to_string(pref_grasp_id) + " > " + std::to_string(corr_grasp_id));
                        std::string corr_to_pref(std::to_string(corr_grasp_id) + " > " + std::to_string(pref_grasp_id));
                        if(write_res < 0 || write_res2 < 0)
                            ROS_ERROR_STREAM("Unable to write transition " << (write_res<0?pref_to_corr:"") << (write_res*write_res2>0?" AND ":"") << (write_res2<0?corr_to_pref:""));
                        else if(write_res == 0 || write_res2 == 0)
                            ROS_WARN_STREAM("Transition " << (write_res==0?pref_to_corr:"") << (write_res+write_res2==0?" AND ":"") << (write_res2==0?corr_to_pref:"") << " was(were) already present in the DB - not added");
                        #if DEBUG
                        else
                            ROS_INFO_STREAM("Written transitions " << pref_to_corr << " and " << corr_to_pref << " in the DB with IDs " << write_res << " and " << write_res2);
                        #endif
                    }
                }
            }
        }
    }
    db_writer_->close_global();
    return true;
}
