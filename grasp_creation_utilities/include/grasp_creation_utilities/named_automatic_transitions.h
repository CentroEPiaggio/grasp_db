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
