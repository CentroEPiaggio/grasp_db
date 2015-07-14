#include "ros/ros.h"
#include <kdl/frames.hpp>
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/databasewriter.h>
#include "grasp_creation_utilities/table_grasp_maker.h"
#include <algorithm>

#define HOW_MANY_ROT 8 // how many rotations for table grasps
#define HOW_MANY_VAR 8 // how many variations for each of the hand grasps
#define STARTING_EE_ID 2
#define OBJECT_ID 10
#define DB_NAME "multi_arm_cylinder.db"
#define NAMED_TRANSITIONS true
#define CYLINDER_HEIGHT 0.2
#define SINGLE_HAND_GRASP_LIMIT 100
#define NUM_VITO 3

/*
Assumptions >>> the database already contains:
- workspaces, geometry, adjacencies, reachability
- the object Cylinder with id OBJECT_ID
- 3 end effectors, [left_hand(1) right_hand(2) table(3)]
- right_hand bottom and side_low grasps, which are also already serialized, with HOW_MANY_VAR variations each
  >> and IDs starting at 101
*/

bool specularize_grasps(uint new_ee_id,std::string new_link_name,std::vector<std::string> new_joint_names,std::vector<uint> specularized_grasps,std::vector<uint> new_grasp_ids,std::vector<std::string> specularized_grasp_names,bool top_bottom,std::string db_name)
{
    databaseMapper db_mapper( db_name );
    
    assert( specularized_grasps.size() == specularized_grasp_names.size() );
    assert( specularized_grasps.size() == new_grasp_ids.size() );

    specularGraspMaker sgm( new_ee_id, new_link_name, new_joint_names, db_name );
    
    for(int i=0; i < specularized_grasps.size(); i++)
    {
        uint grasp_id = specularized_grasps.at(i);
        uint new_grasp_id = new_grasp_ids.at(i);
        std::string new_grasp_name = specularized_grasp_names.at(i);
        
        uint obj_id;
        uint ee_id;
        std::string grasp_name;

        obj_id = std::get<0>(db_mapper.Grasps.at(grasp_id));
        ee_id = std::get<1>(db_mapper.Grasps.at(grasp_id));
        grasp_name = std::get<2>(db_mapper.Grasps.at(grasp_id));
        
        ROS_INFO_STREAM("Converting grasp " << grasp_name << " > " << new_grasp_name << " (" << (i+1) << " out of " << specularized_grasps.size() << ")");

        bool transform_ok;
        transform_ok = sgm.transform_grasp( obj_id, grasp_id, new_grasp_name, top_bottom, new_grasp_id );
        
        if(!transform_ok)
        {
            ROS_FATAL_STREAM("Stopped transformation at grasp " << grasp_name << " > " << new_grasp_name << "!!!");
            return false;
        }
    }
    return true;
}

int add_vitos_in_cylinder_db(std::string db_name = DB_NAME, int num_vito = NUM_VITO)
{
    // CONDITIONS:
        // - table grasps: top and bottom; 8 each
        // - [MUST BE DONE OUTSIDE] right-hand grasps: bottom, side_low; 8 each
        // - specularize left-hand, complete with top and side_high also for right hand
        // - specularize also other right-hands and left-hands (up to NUM_VITO)
        // - named automatic transitions between top -> {bottom, side_low};
        //   bottom -> {top, side_high}; side_high -> {bottom, side_low}; side_low -> {bottom, side_high}

    // Table grasps: Bottom and Top, with HOW_MANY_ROT rotation steps
    tableGraspMaker table_grasps(db_name,HOW_MANY_ROT);
    std::vector<KDL::Frame> table_grasp_frames;
    table_grasp_frames.emplace_back(KDL::Frame(KDL::Vector(0.0,0.0,-1*CYLINDER_HEIGHT/2.0)));
    table_grasp_frames.emplace_back(KDL::Frame(KDL::Vector(0.0,0.0,CYLINDER_HEIGHT/2.0),KDL::Rotation::RPY(M_PI,0.0,0.0)));
    std::vector<std::string> table_grasp_names({"bottom","top"});
    for(int i=0; i<table_grasp_names.size(); i++)
        if(!table_grasps.create_table_grasps(OBJECT_ID, table_grasp_names.at(i), table_grasp_frames.at(i)))
        {
            ROS_FATAL_STREAM("Unable to create \'" << table_grasp_names.at(i) << "\' table grasps!!!");
            return -1;
        }

    // some useful definitions
    std::vector<std::string> top_vector(HOW_MANY_VAR,"top");
    std::vector<std::string> bottom_vector(HOW_MANY_VAR,"bottom");
    std::vector<std::string> side_high_vector(HOW_MANY_VAR,"side_high");
    std::vector<std::string> side_low_vector(HOW_MANY_VAR,"side_low");

    // PART 1
    // Specularize right_hand bottom and side_low into left_hand top and side_high
    uint new_ee_id = 1;
    uint old_ee_id = STARTING_EE_ID;
    std::string new_link_name = "left_hand_palm_link";
    std::vector<std::string> new_joint_names = {"left_hand_synergy_joint"};
    std::vector<uint> specularized_grasps(HOW_MANY_VAR*2);
    std::iota(specularized_grasps.begin(),specularized_grasps.end(),SINGLE_HAND_GRASP_LIMIT*old_ee_id + 1);

    std::vector<std::string> specularized_grasp_names;
    specularized_grasp_names.clear();
    specularized_grasp_names.insert(specularized_grasp_names.begin(),top_vector.begin(),top_vector.end());
    specularized_grasp_names.insert(specularized_grasp_names.begin(),side_high_vector.begin(),side_high_vector.end());
    bool top_bottom = true;

    std::vector<uint> new_grasp_ids(specularized_grasps.size());
    // as I'm specularizing top_bottom, I use higher ids
    std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),SINGLE_HAND_GRASP_LIMIT*new_ee_id + 1 + HOW_MANY_VAR*2);

    if(!specularize_grasps(new_ee_id,new_link_name,new_joint_names,specularized_grasps,new_grasp_ids,specularized_grasp_names,top_bottom,db_name))
    {
        ROS_FATAL_STREAM("Unable to specularize " << specularized_grasps.size() << " grasps from ee_id:" << old_ee_id << " to ee_id:" << new_ee_id << "(" << new_link_name << ") and " << (top_bottom?"":"NOT ") << "inverting top-bottom");
        return -1;
    }
    else
        ROS_INFO_STREAM("Successfully specularized " << specularized_grasps.size() << " grasps from ee_id:" << old_ee_id << " to ee_id:" << new_ee_id << "(" << new_link_name << ") and " << (top_bottom?"":"NOT ") << "inverting top-bottom");

    // PART 2
    // Specularize right_hand bottom and side_low into left_hand bottom and side_low
    specularized_grasp_names.clear();
    specularized_grasp_names.insert(specularized_grasp_names.begin(),bottom_vector.begin(),bottom_vector.end());
    specularized_grasp_names.insert(specularized_grasp_names.begin(),side_low_vector.begin(),side_low_vector.end());
    top_bottom = false;
    // I use ids starting from 1 as I'm not using top_bottom
    std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),SINGLE_HAND_GRASP_LIMIT*new_ee_id + 1);

    if(!specularize_grasps(new_ee_id,new_link_name,new_joint_names,specularized_grasps,new_grasp_ids,specularized_grasp_names,top_bottom,db_name))
    {
        ROS_FATAL_STREAM("Unable to specularize " << specularized_grasps.size() << " grasps from ee_id:" << old_ee_id << " to ee_id:" << new_ee_id << "(" << new_link_name << ") and " << (top_bottom?"":"NOT ") << "inverting top-bottom");
        return -1;
    }
    else
        ROS_INFO_STREAM("Successfully specularized " << specularized_grasps.size() << " grasps from ee_id:" << old_ee_id << " to ee_id:" << new_ee_id << "(" << new_link_name << ") and " << (top_bottom?"":"NOT ") << "inverting top-bottom");

    // PART 3
    // Specularize left_hand bottom and side_low into right_hand top and side_high
    new_ee_id = 2;
    old_ee_id = 1;
    new_link_name = "right_hand_palm_link"
    new_joint_names.swap(std::vector<std::string>({"right_hand_synergy_joint"}));
    std::iota(specularized_grasps.begin(),specularized_grasps.end(),SINGLE_HAND_GRASP_LIMIT*old_ee_id + 1);
    specularized_grasp_names.clear();
    specularized_grasp_names.insert(specularized_grasp_names.begin(),top_vector.begin(),top_vector.end());
    specularized_grasp_names.insert(specularized_grasp_names.begin(),side_high_vector.begin(),side_high_vector.end());
    top_bottom = false;
    // I use ids NOT starting from 1 as I'm not using top_bottom
    std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),SINGLE_HAND_GRASP_LIMIT*new_ee_id + 1 + HOW_MANY_VAR*2);

    if(!specularize_grasps(new_ee_id,new_link_name,new_joint_names,specularized_grasps,new_grasp_ids,specularized_grasp_names,top_bottom,db_name))
    {
        ROS_FATAL_STREAM("Unable to specularize " << specularized_grasps.size() << " grasps from ee_id:" << old_ee_id << " to ee_id:" << new_ee_id << "(" << new_link_name << ") and " << (top_bottom?"":"NOT ") << "inverting top-bottom");
        return -1;
    }
    else
        ROS_INFO_STREAM("Successfully specularized " << specularized_grasps.size() << " grasps from ee_id:" << old_ee_id << " to ee_id:" << new_ee_id << "(" << new_link_name << ") and " << (top_bottom?"":"NOT ") << "inverting top-bottom");

    // PART 4
    // Add as many hands as you want...
    specularized_grasp_names.clear();
    specularized_grasp_names.insert(specularized_grasp_names.begin(),bottom_vector.begin(),bottom_vector.end());
    specularized_grasp_names.insert(specularized_grasp_names.begin(),side_low_vector.begin(),side_low_vector.end());
    specularized_grasp_names.insert(specularized_grasp_names.begin(),top_vector.begin(),top_vector.end());
    specularized_grasp_names.insert(specularized_grasp_names.begin(),side_high_vector.begin(),side_high_vector.end());
    top_bottom = false;
    specularized_grasps.clear();
    specularized_grasps.resize(specularized_grasp_names.size());
    new_grasp_ids.clear();
    new_grasp_ids.resize(specularized_grasp_names.size());

    // HERE choose from which ee to take grasps, and which will the new ee_id be
    std::vector<uint> new_hand_ids;
    std::vector<uint> old_hand_ids;
    int hand_count = 3;
    for(int i=0; i<num_vito; i++)
    {
        old_hand_ids.push_back(2);
        old_hand_ids.push_back(1);
        new_hand_ids.push_back(++count);
        new_hand_ids.push_back(++count);
    }

    assert(new_hand_ids.size() == old_hand_ids.size());

    for(int i=0; i<new_hand_ids.size(); i++)
    {
        new_ee_id = new_hand_ids.at(i);
        old_ee_id = old_hand_ids.at(i);
        // check wether the new hand is a right or left one (opposite of the old one!)
        std::string left_right = (old_ee_id%3 == 1)?"right":"left";
        new_link_name =  left_right + "_hand_palm_link";
        new_joint_names.swap(std::vector<std::string>({left_right + "_hand_synergy_joint"}));
        std::iota(specularized_grasps.begin(),specularized_grasps.end(),SINGLE_HAND_GRASP_LIMIT*old_ee_id + 1);
        std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),SINGLE_HAND_GRASP_LIMIT*new_ee_id + 1);
        if(!specularize_grasps(new_ee_id,new_link_name,new_joint_names,specularized_grasps,new_grasp_ids,specularized_grasp_names,top_bottom,db_name))
        {
            ROS_FATAL_STREAM("Unable to specularize " << specularized_grasps.size() << " grasps from ee_id:" << old_ee_id << " to ee_id:" << new_ee_id << "(" << new_link_name << ") and " << (top_bottom?"":"NOT ") << "inverting top-bottom");
            return -1;
        }
        else
            ROS_INFO_STREAM("Successfully specularized " << specularized_grasps.size() << " grasps from ee_id:" << old_ee_id << " to ee_id:" << new_ee_id << "(" << new_link_name << ") and " << (top_bottom?"":"NOT ") << "inverting top-bottom");
    }

    // Make all transitions based on names
    // // THIS COULD BE DONE WITH THE GEOMETRIC FILTER
    if(NAMED_TRANSITIONS)
    {
        std::vector<std::string> prefixes({"bottom","top","side_high","side_low"});
        std::map<std::string,std::vector<std::string>> correspondences;
        correspondences["bottom"]    = {"top",      "side_high"};
        correspondences["side_low"]  = {"top",      "side_high"};
        correspondences["top"]       = {"bottom",   "side_low"};
        correspondences["side_high"] = {"bottom",   "side_low"};

        namedAutomaticTransitions nat( prefixes, correspondences, db_name );

        bool write_ok = nat.write_transitions();
        if(!write_ok)
        {
            ROS_FATAL_STREAM("Unable to write transitions!!!");
            return -1;
        }
    }

    return 0;
}