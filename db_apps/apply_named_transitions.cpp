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