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
#include <kdl/frames.hpp>
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/databasewriter.h>
#include "grasp_creation_utilities/table_grasp_maker.h"
#include <grasp_creation_utilities/specular_grasp_maker.h>
#include <grasp_creation_utilities/symmetric_grasp_maker.h>
#include <grasp_creation_utilities/named_automatic_transitions.h>
#include <algorithm>

#define HOW_MANY_ROT 16 // how many rotations for table grasps
#define HOW_MANY_VAR 8 // how many variations for each of the hand grasps
#define STARTING_EE_ID 2
#define SOURCE_EE_FRAME "0_right_hand_palm_link"
#define SOURCE_JOINTS {"0_right_hand_synergy_joint"}
#define OBJECT_ID 10
#define DB_NAME "multi_arm_cylinder.db"
#define NAMED_TRANSITIONS true
#define CYLINDER_HEIGHT 0.255
#define SINGLE_HAND_GRASP_LIMIT 100
#define NUM_VITO 3
#define BELT_WP_HEIGHT 0.0
#define TABLE_WP_HEIGHT 0.05
#define BELT_EE_ID 8
#define NO_CONSTRAINT_ID 1
// a small additional height
#define EPS 0.01

/*
Assumptions >>> the database already contains:
- workspaces, geometry, adjacencies, reachability
- the object Cylinder with id OBJECT_ID
- 3 end effectors, [left_hand(1) right_hand(2) table(3)]
- right_hand bottom and sidelow grasps, 1 each, which are also already serialized
  >> and IDs which are 201(bottom) and 201+how_many_var(sidelow)
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

        obj_id = db_mapper.Grasps.at(grasp_id).obj_id;
        ee_id = db_mapper.Grasps.at(grasp_id).ee_id;
        grasp_name = db_mapper.Grasps.at(grasp_id).name;
        
        ROS_INFO_STREAM("Converting grasp " << grasp_name << " > " << new_grasp_name << " (" << (i+1) << " out of " << specularized_grasps.size() << ")");

        bool transform_ok;
        transform_ok = sgm.transform_grasp( obj_id, grasp_id, new_grasp_name, top_bottom, new_grasp_id, NO_CONSTRAINT_ID );
        
        if(!transform_ok)
        {
            ROS_FATAL_STREAM("Stopped transformation at grasp " << grasp_name << " > " << new_grasp_name << "!!!");
            return false;
        }
    }
    return true;
}

uint robot_id_from_ee_id(uint ee_id)
{
    assert(ee_id > 0);
    // this gets already floor-rounded
    return (ee_id-1)/2;
}

int add_vitos_in_cylinder_db(std::string db_name = DB_NAME, int num_vito = NUM_VITO, int how_many_var = HOW_MANY_VAR)
{
    // CONDITIONS:
        // - table grasps: top and bottom; HOW_MANY_ROT each
        // - belt grasps: top and bottom; HOW_MANY_ROT each
        // - [MUST BE DONE OUTSIDE] right-hand grasps: bottom, sidelow; how_many_var each
        // - specularize left-hand, complete with top and sidehigh also for right hand
        // - specularize also other right-hands and left-hands (up to NUM_VITO)
        // - named automatic transitions between top -> {bottom, sidelow};
        //   bottom -> {top, sidehigh}; sidehigh -> {bottom, sidelow}; sidelow -> {bottom, sidehigh}

    // Table grasps: Bottom and Top, with HOW_MANY_ROT rotation steps
    tableGraspMaker table_grasps(db_name,HOW_MANY_ROT,num_vito*2+1,TABLE_WP_HEIGHT);
    std::vector<KDL::Frame> table_grasp_frames;
    table_grasp_frames.emplace_back(KDL::Frame(KDL::Vector(0.0,0.0,-1*CYLINDER_HEIGHT/2.0-EPS)));
    table_grasp_frames.emplace_back(KDL::Frame(KDL::Rotation::RPY(M_PI,0.0,0.0),KDL::Vector(0.0,0.0,CYLINDER_HEIGHT/2.0+EPS)));
    std::vector<std::string> table_grasp_names({"bottom_p_table","top_p_table"});
    for(int i=0; i<table_grasp_names.size(); i++)
        if(!table_grasps.create_table_grasps(OBJECT_ID, table_grasp_names.at(i), table_grasp_frames.at(i),1+i*HOW_MANY_ROT,NO_CONSTRAINT_ID))
        {
            ROS_FATAL_STREAM("Unable to create \'" << table_grasp_names.at(i) << "\' table grasps!!!");
            return -1;
        }
    
    // Belt grasps: Bottom and Top, as the table
    tableGraspMaker belt_grasps(db_name,HOW_MANY_ROT,BELT_EE_ID,BELT_WP_HEIGHT,"belt0_p");
    std::vector<KDL::Frame> belt_grasp_frames;
    belt_grasp_frames.emplace_back(KDL::Frame(KDL::Vector(0.0,0.0,-1*CYLINDER_HEIGHT/2.0-2.0*EPS)));
    belt_grasp_frames.emplace_back(KDL::Frame(KDL::Rotation::RPY(M_PI,0.0,0.0),KDL::Vector(0.0,0.0,CYLINDER_HEIGHT/2.0+2.0*EPS)));
    std::vector<std::string> belt_grasp_names({"bottom_p_belt","top_p_belt"});
    for(int i=0; i<belt_grasp_names.size(); i++)
        if(!belt_grasps.create_table_grasps(OBJECT_ID, belt_grasp_names.at(i), belt_grasp_frames.at(i),BELT_EE_ID*100+1+i*HOW_MANY_ROT,NO_CONSTRAINT_ID))
        {
            ROS_FATAL_STREAM("Unable to create \'" << belt_grasp_names.at(i) << "\' belt grasps!!!");
            return -1;
        }

    // PART 0
    // Generate via symmetry how_many_var-1 new grasps for the right hand
    {
        KDL::Vector z_axis(0,0,1);
        KDL::Frame rotFrame_obj(KDL::Frame::Identity());
        symmetricGraspMaker sgm(STARTING_EE_ID,SOURCE_EE_FRAME,SOURCE_JOINTS,db_name);
        // Reserialize original grasps to make sure they are ok, then go on
        if(!sgm.reserialize_grasp(OBJECT_ID,SINGLE_HAND_GRASP_LIMIT*STARTING_EE_ID + 1) || !sgm.reserialize_grasp(OBJECT_ID,SINGLE_HAND_GRASP_LIMIT*STARTING_EE_ID + 1))
            return -1;
        
        std::vector<uint> new_grasp_ids(how_many_var - 1);

        // rotate bottom grasp
        std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),SINGLE_HAND_GRASP_LIMIT*STARTING_EE_ID + 2);
        if(!sgm.transform_grasp(OBJECT_ID,SINGLE_HAND_GRASP_LIMIT*STARTING_EE_ID + 1,"bottom_right",new_grasp_ids,how_many_var,z_axis,rotFrame_obj,NO_CONSTRAINT_ID))
            return -1;
        // rotate sidelow grasp
        std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),SINGLE_HAND_GRASP_LIMIT*STARTING_EE_ID + how_many_var + 2);
        if(!sgm.transform_grasp(OBJECT_ID,SINGLE_HAND_GRASP_LIMIT*STARTING_EE_ID + how_many_var + 1,"sidelow_right",new_grasp_ids,how_many_var,z_axis,rotFrame_obj,NO_CONSTRAINT_ID))
            return -1;
    }

    // some useful definitions
    std::vector<std::string> top_vector_right(how_many_var,"top_right");
    std::vector<std::string> bottom_vector_right(how_many_var,"bottom_right");
    std::vector<std::string> sidehigh_vector_right(how_many_var,"sidehigh_right");
    std::vector<std::string> sidelow_vector_right(how_many_var,"sidelow_right");
    std::vector<std::string> top_vector_left(how_many_var,"top_left");
    std::vector<std::string> bottom_vector_left(how_many_var,"bottom_left");
    std::vector<std::string> sidehigh_vector_left(how_many_var,"sidehigh_left");
    std::vector<std::string> sidelow_vector_left(how_many_var,"sidelow_left");

    // PART 1
    // Specularize right_hand bottom and sidelow into left_hand top and sidehigh
    uint new_ee_id = 1;
    uint old_ee_id = STARTING_EE_ID;
    std::string new_link_name = std::to_string( robot_id_from_ee_id(new_ee_id) ) + "_left_hand_palm_link";
    std::vector<std::string> new_joint_names = {std::to_string( robot_id_from_ee_id(new_ee_id) ) + "_left_hand_synergy_joint"};
    std::vector<uint> specularized_grasps(how_many_var*2);
    std::iota(specularized_grasps.begin(),specularized_grasps.end(),SINGLE_HAND_GRASP_LIMIT*old_ee_id + 1);

    std::vector<std::string> specularized_grasp_names;
    specularized_grasp_names.clear();
    specularized_grasp_names.insert(specularized_grasp_names.end(),top_vector_left.begin(),top_vector_left.end());
    specularized_grasp_names.insert(specularized_grasp_names.end(),sidehigh_vector_left.begin(),sidehigh_vector_left.end());
    bool top_bottom = true;

    std::vector<uint> new_grasp_ids(specularized_grasps.size());
    // as I'm specularizing top_bottom, I use higher ids
    std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),SINGLE_HAND_GRASP_LIMIT*new_ee_id + 1 + how_many_var*2);

    if(!specularize_grasps(new_ee_id,new_link_name,new_joint_names,specularized_grasps,new_grasp_ids,specularized_grasp_names,top_bottom,db_name))
    {
        ROS_FATAL_STREAM("Unable to specularize " << specularized_grasps.size() << " grasps from ee_id:" << old_ee_id << " to ee_id:" << new_ee_id << "(" << new_link_name << ") and " << (top_bottom?"":"NOT ") << "inverting top-bottom");
        return -1;
    }
    else
        ROS_INFO_STREAM("Successfully specularized " << specularized_grasps.size() << " grasps from ee_id:" << old_ee_id << " to ee_id:" << new_ee_id << "(" << new_link_name << ") and " << (top_bottom?"":"NOT ") << "inverting top-bottom");

    // PART 2
    // Specularize right_hand bottom and sidelow into left_hand bottom and sidelow
    specularized_grasp_names.clear();
    specularized_grasp_names.insert(specularized_grasp_names.end(),bottom_vector_left.begin(),bottom_vector_left.end());
    specularized_grasp_names.insert(specularized_grasp_names.end(),sidelow_vector_left.begin(),sidelow_vector_left.end());
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
    // Specularize left_hand bottom and sidelow into right_hand top and sidehigh
    new_ee_id = 2;
    old_ee_id = 1;
    new_link_name = std::to_string( robot_id_from_ee_id(new_ee_id) ) + "_right_hand_palm_link";
    new_joint_names=std::vector<std::string>({std::to_string( robot_id_from_ee_id(new_ee_id) ) + "_right_hand_synergy_joint"});
    std::iota(specularized_grasps.begin(),specularized_grasps.end(),SINGLE_HAND_GRASP_LIMIT*old_ee_id + 1);
    specularized_grasp_names.clear();
    specularized_grasp_names.insert(specularized_grasp_names.end(),top_vector_right.begin(),top_vector_right.end());
    specularized_grasp_names.insert(specularized_grasp_names.end(),sidehigh_vector_right.begin(),sidehigh_vector_right.end());
    top_bottom = true;
    std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),SINGLE_HAND_GRASP_LIMIT*new_ee_id + 1 + how_many_var*2);

    if(!specularize_grasps(new_ee_id,new_link_name,new_joint_names,specularized_grasps,new_grasp_ids,specularized_grasp_names,top_bottom,db_name))
    {
        ROS_FATAL_STREAM("Unable to specularize " << specularized_grasps.size() << " grasps from ee_id:" << old_ee_id << " to ee_id:" << new_ee_id << "(" << new_link_name << ") and " << (top_bottom?"":"NOT ") << "inverting top-bottom");
        return -1;
    }
    else
        ROS_INFO_STREAM("Successfully specularized " << specularized_grasps.size() << " grasps from ee_id:" << old_ee_id << " to ee_id:" << new_ee_id << "(" << new_link_name << ") and " << (top_bottom?"":"NOT ") << "inverting top-bottom");

    // PART 4
    // Add as many hands as you want...
    top_bottom = false;

    // HERE choose from which ee to take grasps, and which will the new ee_id be
    std::vector<uint> new_hand_ids;
    std::vector<uint> old_hand_ids;
    int hand_count = 2;
    
    // only create grasps for hands which are in the DB
    databaseMapper db_mapper( db_name );
    
    for(int i=0; i<num_vito-1; i++)
    {
        if(db_mapper.EndEffectors.count(++hand_count))
        {
            old_hand_ids.push_back(2);
            new_hand_ids.push_back(hand_count);
        }
        if(db_mapper.EndEffectors.count(++hand_count))
        {
            old_hand_ids.push_back(1);
            new_hand_ids.push_back(hand_count);
        }
    }

    assert(new_hand_ids.size() == old_hand_ids.size());

    for(int i=0; i<new_hand_ids.size(); i++)
    {
        new_ee_id = new_hand_ids.at(i);
        old_ee_id = old_hand_ids.at(i);
        // check wether the new hand is a right or left one (opposite of the old one!)
        std::string left_right = (old_ee_id%3 == 1)?"right":"left";
        specularized_grasp_names.clear();
        if(left_right.at(0) == 'l')
        {
            specularized_grasp_names.insert(specularized_grasp_names.end(),bottom_vector_left.begin(),bottom_vector_left.end());
            specularized_grasp_names.insert(specularized_grasp_names.end(),sidelow_vector_left.begin(),sidelow_vector_left.end());
            specularized_grasp_names.insert(specularized_grasp_names.end(),top_vector_left.begin(),top_vector_left.end());
            specularized_grasp_names.insert(specularized_grasp_names.end(),sidehigh_vector_left.begin(),sidehigh_vector_left.end());
        }
        else
        {
            specularized_grasp_names.insert(specularized_grasp_names.end(),bottom_vector_right.begin(),bottom_vector_right.end());
            specularized_grasp_names.insert(specularized_grasp_names.end(),sidelow_vector_right.begin(),sidelow_vector_right.end());
            specularized_grasp_names.insert(specularized_grasp_names.end(),top_vector_right.begin(),top_vector_right.end());
            specularized_grasp_names.insert(specularized_grasp_names.end(),sidehigh_vector_right.begin(),sidehigh_vector_right.end());
        }
        specularized_grasps.clear();
        specularized_grasps.resize(specularized_grasp_names.size());
        new_grasp_ids.clear();
        new_grasp_ids.resize(specularized_grasp_names.size());
        new_link_name = std::to_string( robot_id_from_ee_id(new_ee_id) ) + "_" + left_right + "_hand_palm_link";
        new_joint_names=std::vector<std::string>({std::to_string( robot_id_from_ee_id(new_ee_id) ) + "_" + left_right + "_hand_synergy_joint"});
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
        std::vector<std::string> prefixes({"bottom_l","top_l","sidehigh_l","sidelow_l","bottom_r","top_r","sidehigh_r","sidelow_r","bottom_p","top_p"});
        
        std::map<std::string,std::vector<std::string>> correspondences;
        correspondences["bottom_l"] = {"top", "sidehigh"};
        correspondences["top_l"] = {"bottom", "sidelow"};
        correspondences["sidehigh_l"] = {"bottom", "sidelow"};
        correspondences["sidelow_l"] = {"top", "sidehigh"};
        correspondences["bottom_r"] = {"top", "sidehigh"};
        correspondences["top_r"] = {"bottom", "sidelow"};
        correspondences["sidehigh_r"] = {"bottom", "sidelow"};
        correspondences["sidelow_r"] = {"top", "sidehigh"};
        correspondences["bottom_p"] = {"top_r", "sidehigh_r", "top_l", "sidehigh_l"};
        correspondences["top_p"] = {"bottom_r", "sidelow_r", "bottom_l", "sidelow_l"};

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