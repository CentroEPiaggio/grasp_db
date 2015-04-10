#include "named_automatic_transitions.h"
#include <string>

#define DEBUG 1

namedAutomaticTransitions::namedAutomaticTransitions(std::string db_name):db_name_(db_name)
{
  if (node.getParam("dual_manipulation_grasp_db", params))
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
    parseSingleParameter(params["tree_composition"],corr_tmp[pref],pref,1);
    
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
  
  for(auto grasp:db_mapper_->Grasps)
  {
    uint grasp_id = grasp.first;

    uint obj_id = std::get<0>(grasp.second);
    uint ee_id = std::get<1>(grasp.second);
    std::string grasp_name = std::get<2>(grasp.second);

    for(auto pref:prefixes_)
    {
      if(compare_and_store(grasp_name,pref,grasp_id_from_prefix,grasp_id))
	ee_id_from_prefix[pref].push_back(ee_id);
      
      // do the same for each correspondence
      for(auto corr:correspondences_.at(pref))
      {
	if(compare_and_store(grasp_name,corr,grasp_id_from_prefix,grasp_id))
	  ee_id_from_prefix[corr].push_back(ee_id);
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
    std::cout << "grasp prefix " << ee.first << " -> Ee IDs: ";
    for(auto id:ee.second)
      std::cout << id << ", ";
    std::cout << std::endl;
  }
#endif
  
  for(auto pref:prefixes_)
    for(auto corr:correspondences_.at(pref))
      for(int i1=0; i1<ee_id_from_prefix.at(pref).size(); i1++)
      {
	uint pref_ee_id = ee_id_from_prefix.at(pref).at(i1);
	
	for(int i2=0; i2<ee_id_from_prefix.at(corr).size(); i2++)
	{
	  uint corr_ee_id = ee_id_from_prefix.at(corr).at(i2);
	
	  if(pref_ee_id != corr_ee_id)
	  {
	    uint pref_grasp_id = grasp_id_from_prefix.at(pref).at(i1);
	    uint corr_grasp_id = grasp_id_from_prefix.at(corr).at(i2);
#if DEBUG
	    int write_res = db_writer_->writeNewTransition(pref_grasp_id,corr_grasp_id);
	    std::string pref_to_corr(std::to_string(pref_grasp_id) + " > " + std::to_string(corr_grasp_id));
	    if(write_res < 0)
	      ROS_ERROR_STREAM("Unable to write transition " << pref_to_corr);
	    else if(write_res == 0)
	      ROS_WARN_STREAM("Transition " << pref_to_corr << " was already present in the DB - not added");
	    else
	      ROS_INFO_STREAM("Written transition " << pref_to_corr << " in the DB with ID " << write_res);
#endif
	  }
	}
      }
  
  return true;
}
