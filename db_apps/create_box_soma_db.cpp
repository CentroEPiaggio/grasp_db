/*********************************************************************
 * 
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Alessandro Settimi, Hamal Marino, Mirko Ferrati
 *  Centro di Ricerca "E. Piaggio", University of Pisa
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
#include "ros/package.h"
#include <ctime>
#include <kdl/frames.hpp>
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/databasewriter.h>
#include <dual_manipulation_shared/serialization_utils.h>
#include <dual_manipulation_shared/grasp_trajectory.h>
#include <grasp_creation_utilities/table_grasp_maker.h>
#include <grasp_creation_utilities/specular_grasp_maker.h>
#include <grasp_creation_utilities/symmetric_grasp_maker.h>
#include <grasp_creation_utilities/named_automatic_transitions.h>
#include <grasp_creation_utilities/create_db_utils.h>
#include <algorithm>

#define BOX_SIDE_A 0.175 // length along x-axis
#define BOX_SIDE_B 0.112 // length along y-axis
#define BOX_SIDE_C 0.046 // length along z-axis
#define DB_POSTFIX "box_soma_"
#define EMPTY_DB "empty"

#define TINO_WP_HEIGHT 0.0
#define TABLE_WP_HEIGHT 0.05
#define EPS 0.01 // a small additional height
#define NAMED_TRANSITIONS 1
// constraint id's
#define NO_CONSTRAINT_TYPE_ID 1
#define EDGE_CONSTRAINT_TYPE_ID 2
#define WALL_CONSTRAINT_TYPE_ID 3
#define NO_CONSTRAINT_ID 1
#define EDGE_CONSTRAINT_1 2
#define EDGE_CONSTRAINT_2 3

// grasp ID-ing
#define OBJ_GRASP_FACTOR dual_manipulation::shared::OBJ_GRASP_FACTOR
#define EE_GRASP_FACTOR 200 // MUST BE: #ee's * EE_GRASP_FACTOR < OBJ_GRASP_FACTOR
#define EC_GRASP_FACTOR 40  // MUST BE: #ec's * EC_GRASP_FACTOR < EE_GRASP_FACTOR
#define HOW_MANY_ROT 1 // how many rotations for non-prehensile grasps // MUST BE: #non-prehensile_grasps * HOW_MANY_ROT < EC_GRASP_FACTOR
#define HOW_MANY_VAR 1 // how many variations for prehensile grasps // MUST BE: #prehensile_grasps * 2 * HOW_MANY_VAR < EC_GRASP_FACTOR

/**
 * Assumptions
 *  - a file named ${EMPTY_DB}.db exists with tables already created but empty
 *  - for each object to be added in the database, there is a bottom grasp of each type (written in ee_grasp_names) already serialized using a *right_hand* in the folder "object(OBJECT_ID)", with IDs (OBJ_GRASP_FACTOR-1 .. OBJ_GRASP_FACTOR-ee_grasp_names.at(ee).size())
 *  - movable end-effectors are right_hands if they have an even ID, left_hands if the ID is odd
 *  - database grasp id rule will be: obj_id*OBJ_GRASP_FACTOR + ee_id*EE_GRASP_FACTOR + ec_id*EC_GRASP_FACTOR + grasp_id, where grasp_id varies between 1 and
 *      - HOW_MANY_ROT for non-prehensile grasps
 *      - HOW_MANY_VAR*2 for prehensile grasps
 */

// DATA that will be stored in the database

// OBJECTS
std::map<object_id,std::pair<std::string,std::string>> object_list = {
    {401,{"boxSoma","package://dual_manipulation_grasp_db/object_meshes/Box_175x112x46.stl"}}
};
std::map<object_id,KDL::Frame> object_centers = {
    {401,KDL::Frame::Identity()}
};

// END-EFFECTORS
std::vector<endeffector_id> ee_ids = {1,2,3}; // to stay as generic as possible
std::vector<std::string> ee_name = {"left_hand","right_hand","table"};
std::vector<bool> ee_movable = {true,true,false}; // can move
std::vector<bool> ee_prehensile = {true,true,false}; // can be actuated during grasping
std::vector<std::string> ee_link = {"left_hand_palm_link","right_hand_palm_link",""};
std::vector<std::vector<std::string>> ee_prehension_joints = {
    {"left_hand_synergy_joint"},
    {"right_hand_synergy_joint"},
    {}
};
std::map<endeffector_id,std::vector<workspace_id>> reachability={
    {1,{1,2}}, // left_hand
    {2,{2,3}}, // right_hand
    {3,{1,2,3}} // table
};

// NON-PREHENSILE END-EFFECTORS
std::vector<std::vector<KDL::Frame>> ee_nonprehensile_grasps = {
    {},
    {},
    {   KDL::Frame(KDL::Rotation::RPY(0.0,0.0,M_PI/2.0), KDL::Vector(0.0,0.0,-1*BOX_SIDE_C/2.0-EPS)), // table
        KDL::Frame(KDL::Rotation::RPY(M_PI/2.0,0.0,0.0),KDL::Vector(0.0,-1*BOX_SIDE_B/2.0+EPS,0.0)),
        KDL::Frame(KDL::Rotation::RPY(0.0,-1*M_PI/2.0,0.0),KDL::Vector(-1*BOX_SIDE_A/2.0+EPS,0.0,0.0)),
        KDL::Frame(KDL::Rotation::RPY(M_PI,0.0,0.0),KDL::Vector(0.0,0.0,BOX_SIDE_C/2.0+EPS)),
        KDL::Frame(KDL::Rotation::RPY(-1*M_PI/2.0,0.0,0.0),KDL::Vector(0.0,BOX_SIDE_B/2.0+EPS,0.0)),
        KDL::Frame(KDL::Rotation::RPY(0.0,M_PI/2.0,0.0),KDL::Vector(BOX_SIDE_A/2.0+EPS,0.0,0.0))    }
};
std::vector<std::vector<std::string>> ee_grasp_names = {
    {   "sideA_",       "sideB_"    }, // left_hand
    {   "sideA_",       "sideB_"    }, // right_hand
    {   "sideC_bottom", "sideB_bottom", "sideA_bottom",
        "sideC_top",    "sideB_top",    "sideA_top" }  // table
};
std::vector<std::vector<std::vector<constraint_id>>> ee_grasp_constraint = {
    {   {NO_CONSTRAINT_ID}, {NO_CONSTRAINT_ID}  }, // left_hand
    {   {NO_CONSTRAINT_ID}, {NO_CONSTRAINT_ID}  }, // right_hand
    {   {NO_CONSTRAINT_ID,EDGE_CONSTRAINT_1, EDGE_CONSTRAINT_2},{NO_CONSTRAINT_ID},{NO_CONSTRAINT_ID},
        {NO_CONSTRAINT_ID,EDGE_CONSTRAINT_1, EDGE_CONSTRAINT_2},{NO_CONSTRAINT_ID},{NO_CONSTRAINT_ID} }  // table
};

// WORKSPACES
// this vector and WS_Y_MIN, WS_Y_MAX define the workspaces geometry (only for rectangular workspaces all with the same Y bounds)
std::vector<double> ws_x_min({-1.7,-1.7,-1.7});
std::vector<double> ws_x_max({0.0,0.0,0.0}); // TODO: Adjust for real ws's
std::vector<double> ws_y_min({-0.6,-0.15,0.15});
std::vector<double> ws_y_max({-0.15,0.15,0.6});
std::vector<double> ws_z_min({0.06,0.06,0.06});
std::vector<double> ws_z_max({0.65,0.65,0.65});
// adjacencies monolateral information is enough
std::map<workspace_id,std::vector<workspace_id>> adjacency = {
    {1,{2}},
    {2,{3}}
};

// ENVIRONMENTAL CONSTRAINTS
std::vector<constraint_type> ec_type_ids = {NO_CONSTRAINT_TYPE_ID,EDGE_CONSTRAINT_TYPE_ID,WALL_CONSTRAINT_TYPE_ID}; // to stay as generic as possible
std::vector<std::string> ec_type_name = {"None","Edge","Wall"};

std::map<constraint_id, constraint_type> ec_type = {    {NO_CONSTRAINT_ID, NO_CONSTRAINT_TYPE_ID},
                                                        {EDGE_CONSTRAINT_1,EDGE_CONSTRAINT_TYPE_ID},
                                                        {EDGE_CONSTRAINT_2, EDGE_CONSTRAINT_TYPE_ID}
};

std::map<constraint_id, std::string> ec_names = {{NO_CONSTRAINT_ID, "NOCONSTRAINT"},
                                                {EDGE_CONSTRAINT_1, "CLOSETABLEEDGE"},
                                                {EDGE_CONSTRAINT_2, "RIGHTTABLEEDGE"}
};
                                                        
std::map<constraint_id, KDL::Frame> ec_poses = {{NO_CONSTRAINT_ID, KDL::Frame(KDL::Vector(-0.85,0.0,0.0))},
                                                {EDGE_CONSTRAINT_1, KDL::Frame(KDL::Rotation::RPY(0.0,0.0,M_PI/2.0), KDL::Vector(-0.45,0.0,0.0))},
                                                {EDGE_CONSTRAINT_2, KDL::Frame(KDL::Rotation::RPY(0.0,0.0,M_PI), KDL::Vector(-0.85,0.6,0.0))}
};
                                                
std::map<constraint_id, std::pair<KDL::Twist, KDL::Twist>> ec_bounds = { 
    {NO_CONSTRAINT_ID,{KDL::Twist(KDL::Vector(0.0,0.0,0.0), KDL::Vector(0.0,0.0,0.0)), KDL::Twist(KDL::Vector(0.0,0.0,0.0), KDL::Vector(0.0,0.0,0.0))}},
    {EDGE_CONSTRAINT_1,{KDL::Twist(KDL::Vector(-0.6,0.0,0.0), KDL::Vector(0.0,0.0,0.0)), KDL::Twist(KDL::Vector(0.6,0.0,0.0), KDL::Vector(0.0,0.0,0.0))}},
    {EDGE_CONSTRAINT_2,{KDL::Twist(KDL::Vector(-0.40,0.0,0.0), KDL::Vector(0.0,0.0,0.0)), KDL::Twist(KDL::Vector(0.4,0.0,0.0), KDL::Vector(0.0,0.0,0.0))}}
};
std::map<constraint_id,std::vector<workspace_id>> ec_reachability={
    {NO_CONSTRAINT_ID,      {1,2,3}},
    {EDGE_CONSTRAINT_1,    {1,2,3}},
    {EDGE_CONSTRAINT_2,    {3}}
};

// bi-lateral information needed // not used for now
std::map<constraint_type,std::vector<constraint_type>> ec_adjacency = {
    {NO_CONSTRAINT_TYPE_ID,      {NO_CONSTRAINT_TYPE_ID,EDGE_CONSTRAINT_TYPE_ID,WALL_CONSTRAINT_TYPE_ID}},
    {EDGE_CONSTRAINT_TYPE_ID,    {NO_CONSTRAINT_TYPE_ID}},
    {WALL_CONSTRAINT_TYPE_ID,    {NO_CONSTRAINT_TYPE_ID}}
};



inline grasp_id computeGraspId(object_id obj_id, endeffector_id ee_id, constraint_id ec_id, grasp_id id_offset)
{
    return obj_id*OBJ_GRASP_FACTOR + ee_id*EE_GRASP_FACTOR + ec_id*EC_GRASP_FACTOR + id_offset;
}

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Grasp db| -> create box soma db "<<std::endl;
    std::cout<<std::endl;
    
    assert(ee_name.size()*EE_GRASP_FACTOR < OBJ_GRASP_FACTOR);
    assert(ec_type_name.size()*EC_GRASP_FACTOR < EE_GRASP_FACTOR);
    for(int i=0; i<ee_name.size(); ++i)
    {
        assert(ee_grasp_names.at(i).size()*(ee_prehensile.at(i)?HOW_MANY_VAR:HOW_MANY_ROT) < EC_GRASP_FACTOR);
    }
    
    ros::init(argc, argv, "create_box_soma_db");
    // get current date/time to use in the naming of the full DB
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [15];
    std::time(&rawtime);
    timeinfo = std::localtime(&rawtime);
    std::strftime(buffer,15,"%Y%m%d_%H%M",timeinfo);
    
    // copy empty.db to a new full_{NOW}.db
    std::string path = ros::package::getPath("dual_manipulation_grasp_db");
    std::string new_db_name(std::string("full_") + DB_POSTFIX + std::string(buffer,15));
    std::string command = "cp " + path + "/" + EMPTY_DB + ".db " + path + "/" + new_db_name + "\n";
    system(command.c_str());
    
    databaseWriter db_writer(new_db_name);
    
    db_writer.open_global();
    
    int ret = writeGlobalDatabaseInformation(db_writer, ws_x_min, ws_x_max, ws_y_min, ws_y_max, ws_z_min, ws_z_max, adjacency, ee_ids, ee_name, ee_movable, ee_prehensile, ee_link, ee_prehension_joints, reachability, ec_type_ids, ec_type_name, ec_reachability, ec_adjacency, ec_names, ec_type, ec_poses, ec_bounds );
    if(ret < 0)
    {
        ROS_ERROR_STREAM("writeGlobalDatabaseInformation returned the error code " << ret);
        return ret;
    }
    
    // for each object...
    for(auto obj:object_list)
    {
        // write object information
        object_id obj_id = obj.first;
        std::string obj_name = obj.second.first;
        std::string obj_mesh = obj.second.second;
        KDL::Frame obj_frame = object_centers.at(obj_id);
        db_writer.writeNewObject(obj_id,obj_name,obj_mesh,obj_frame);
        
        // for each end-effector
        for(int i=0; i<ee_ids.size(); i++)
        {
            endeffector_id ee_id = ee_ids.at(i);
            bool ee_is_right = ((ee_id % 2) == 0);
            
            for(grasp_id grasp_ind = 0; grasp_ind < ee_grasp_names.at(i).size(); ++grasp_ind)
            {
                assert(ee_grasp_names.at(i).size() == ee_grasp_constraint.at(i).size());
                std::string grasp_prefix = ee_grasp_names.at(i).at(grasp_ind);
                for(constraint_id ec_id:ee_grasp_constraint.at(i).at(grasp_ind))
                {
                    // if prehensile, generate all grasps, starting from the exemplary one already serialized
                    if(ee_prehensile.at(i))
                    {
                        // make the first grasp from the example one already saved on disk
                        // first, create the entry in the database
                        grasp_id tmp_grasp_id = OBJ_GRASP_FACTOR-1-grasp_ind;
                        grasp_id tmp_grasp_id2 = tmp_grasp_id;
                        // for a left hand, just add the right_hand grasp
                        db_writer.writeNewGrasp(tmp_grasp_id,obj_id,ee_id,"temporary_grasp_right");
                        db_writer.close_global();
                        // for a right hand, also make the right_hand grasp specular, then perform the same actions using the left_hand grasp
                        if(ee_is_right)
                        {
                            bool top_bottom = false;
                            tmp_grasp_id2-=ee_grasp_names.size();
                            specularize_grasps(ee_id,ee_link.at(i),ee_prehension_joints.at(i),{tmp_grasp_id},{tmp_grasp_id2},{ec_id},{"temporary_grasp_left"},top_bottom,new_db_name);
                        }
                        
                        // obtain, via specularity, a top and a bottom grasp
                        bool top_bottom = true;
                        grasp_id grasp_id_ = (2*grasp_ind)*HOW_MANY_VAR + 1;
                        specularize_grasps(ee_id,ee_link.at(i),ee_prehension_joints.at(i),{tmp_grasp_id2},{computeGraspId(obj_id,ee_id,ec_id,grasp_id_)},{ec_id},{grasp_prefix + "top_" + ee_name.at(i) + "_" + std::to_string(grasp_id_)},top_bottom,new_db_name);
                        grasp_id_ = (2*grasp_ind+1)*HOW_MANY_VAR + 1;
                        top_bottom = false;
                        specularize_grasps(ee_id,ee_link.at(i),ee_prehension_joints.at(i),{tmp_grasp_id2},{computeGraspId(obj_id,ee_id,ec_id,grasp_id_)},{ec_id},{grasp_prefix + "bottom_" + ee_name.at(i) + "_" + std::to_string(grasp_id_)},top_bottom,new_db_name);
                        
                        // Generate via axial symmetry HOW_MANY_VAR-1 new grasps for current end-effector
                        KDL::Vector z_axis(0,0,1);
                        KDL::Frame rotFrame_obj(KDL::Frame::Identity());
                        symmetricGraspMaker sgm(ee_id,ee_link.at(i),ee_prehension_joints.at(i),new_db_name);
                        std::vector<uint> new_grasp_ids(HOW_MANY_VAR - 1);
                        // - use symmetry to rotate top grasp
                        grasp_id_ = (2*grasp_ind)*HOW_MANY_VAR + 1;
                        std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),computeGraspId(obj_id,ee_id,ec_id,grasp_id_+1));
                        if(!sgm.transform_grasp(obj_id,computeGraspId(obj_id,ee_id,ec_id,grasp_id_),grasp_prefix + "top_" + ee_name.at(i),new_grasp_ids,HOW_MANY_VAR,z_axis,rotFrame_obj,ec_id))
                            return -1;
                        // - use symmetry to rotate bottom grasp
                        grasp_id_ = (2*grasp_ind+1)*HOW_MANY_VAR + 1;
                        std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),computeGraspId(obj_id,ee_id,ec_id,grasp_id_+1));
                        if(!sgm.transform_grasp(obj_id,computeGraspId(obj_id,ee_id,ec_id,grasp_id_),grasp_prefix + "bottom_" + ee_name.at(i),new_grasp_ids,HOW_MANY_VAR,z_axis,rotFrame_obj,ec_id))
                            return -1;
                        
                        // last, delete the temporary grasp entry
                        db_writer.open_global();
                        if(ee_is_right)
                            db_writer.deleteGrasp(tmp_grasp_id2);
                        db_writer.deleteGrasp(tmp_grasp_id);
                    }
                    // if movable, generate all grasps as a support surface (like for a table), using the appropriate link
                    else if(ee_movable.at(i))
                    {
                        db_writer.close_global();
                        
                        double wp_height = TINO_WP_HEIGHT;
                        std::string ee_frame_id = ee_link.at(i);
                        
                        tableGraspMaker tgm_grasps(new_db_name,HOW_MANY_ROT,ee_id,wp_height,ee_frame_id);
                        assert(ee_nonprehensile_grasps.at(i).size() == ee_grasp_names.at(i).size());
                        if(!tgm_grasps.create_table_grasps(obj_id, ee_grasp_names.at(i).at(grasp_ind) + "_" + ee_name.at(i), ee_nonprehensile_grasps.at(i).at(grasp_ind),computeGraspId(obj_id,ee_id,ec_id,1 + grasp_ind*HOW_MANY_ROT), ec_id))
                        {
                            ROS_FATAL_STREAM("Unable to create \'" << ee_grasp_names.at(i).at(grasp_ind) + "_" + ee_name.at(i) << "\' grasps!!!");
                            return -1;
                        }
                        
                        db_writer.open_global();
                    }
                    // if table, generate all grasps as a support surface
                    else
                    {
                        db_writer.close_global();
                        
                        double wp_height = TABLE_WP_HEIGHT;
                        std::string ee_frame_id = "world";
                        
                        tableGraspMaker tgm_grasps(new_db_name,HOW_MANY_ROT,ee_id,wp_height,ee_frame_id);
                        assert(ee_nonprehensile_grasps.at(i).size() == ee_grasp_names.at(i).size());
                        if(!tgm_grasps.create_table_grasps(obj_id, ee_grasp_names.at(i).at(grasp_ind) + "_" + ee_name.at(i), ee_nonprehensile_grasps.at(i).at(grasp_ind),computeGraspId(obj_id,ee_id,ec_id,1 + grasp_ind*HOW_MANY_ROT), ec_id))
                        {
                            ROS_FATAL_STREAM("Unable to create \'" << ee_grasp_names.at(i).at(grasp_ind) + "_" + ee_name.at(i) << "\' grasps!!!");
                            return -1;
                        }
                        
                        db_writer.open_global();
                    }
                }
            }
        }
    }
    
    db_writer.close_global();
    
    // Make all transitions based on names
    // // THIS COULD BE DONE WITH THE GEOMETRIC FILTER
    #if NAMED_TRANSITIONS>0
    std::vector<std::string> prefixes({ "sideA_bottom_table","sideA_top_table","sideA_bottom_left","sideA_top_left","sideA_bottom_right","sideA_top_right",
                                        "sideB_bottom_table","sideB_top_table","sideB_bottom_left","sideB_top_left","sideB_bottom_right","sideB_top_right",
                                        "sideC_bottom_table","sideC_top_table"});
    std::map<std::string,std::vector<std::string>> correspondences;
    // allowed sideA : table > hands; hands > [table, hands]
    correspondences["sideA_bottom_table"] = {"sideA_top_left","sideA_top_right"};
    correspondences["sideA_top_table"] = {"sideA_bottom_left","sideA_bottom_right"};
    correspondences["sideA_bottom_left"] = {"sideA_top"}; // both table and other hand(s)
    correspondences["sideA_top_left"] = {"sideA_bottom"};
    correspondences["sideA_bottom_right"] = {"sideA_top"};
    correspondences["sideA_top_right"] = {"sideA_bottom"};
    // allowed sideB : table > hands; hands > table
    correspondences["sideB_bottom_table"] = {"sideB_top_left","sideB_top_right"};
    correspondences["sideB_top_table"] = {"sideB_bottom_left","sideB_bottom_right"};
    correspondences["sideB_bottom_left"] = {"sideB_top_table"}; // only table: sideB is too small to do a grasp exchange
    correspondences["sideB_top_left"] = {"sideB_bottom_table"};
    correspondences["sideB_bottom_right"] = {"sideB_top_table"};
    correspondences["sideB_top_right"] = {"sideB_bottom_table"};
    // allowed sideC : table (no constraint) > table (no or edge constraint); table (edge) > hands
    // NOTE 1: this will also add transitions from table (edge) to table (no constraint)...
    // NOTE 2: this will also add transitions from table (no constraint) to hands, that will always fail...
    correspondences["sideC_bottom_table"] = {   "sideC_bottom_table",
                                                "sideA_top_left","sideA_top_right","sideA_bottom_left","sideA_bottom_right", "sideB_top_left","sideB_top_right","sideB_bottom_left","sideB_bottom_right"};
    correspondences["sideC_top_table"] =    {   "sideC_top_table",
                                                "sideA_top_left","sideA_top_right","sideA_bottom_left","sideA_bottom_right", "sideB_top_left","sideB_top_right","sideB_bottom_left","sideB_bottom_right"};
    
    namedAutomaticTransitions nat( prefixes, correspondences, new_db_name );
    
    bool write_ok = nat.write_transitions();
    if(!write_ok)
    {
        ROS_FATAL_STREAM("Unable to write transitions!!!");
        return -1;
    }
    
    // delete unwanted transitions
    // - between non-movable ee's, starting from ec_id == 2 (UNKNOWN type)
    // - between non-movable ee's with (name == "sideC*" and ec_id == 1) and movable ee's
    {
        databaseMapper db_mapper(new_db_name);
        databaseWriter db_writer_eraser(new_db_name);
        db_writer_eraser.open_global();
        
        int total_transitions = 0;
        int removed_transitions = 0;
        for(auto& trans:db_mapper.Grasp_transitions)
        {
            total_transitions += trans.second.size();
            grasp_id source_id = trans.first;
            endeffector_id source_ee = db_mapper.Grasps.at(source_id).ee_id;
            bool source_movable = db_mapper.EndEffectors.at( source_ee ).movable;
            constraint_id source_ec = db_mapper.Grasps.at(source_id).ec_id;
            std::string source_name = db_mapper.Grasps.at(source_id).name;
            std::string sub_str("sideC");
            bool is_source_sideC = (source_name.compare(0,sub_str.length(),sub_str) == 0);
            
            // in case starting from a movable, all transitions should be ok
            if(source_movable)
                continue;
            
            for(grasp_id target_id:trans.second)
            {
                endeffector_id target_ee = db_mapper.Grasps.at(target_id).ee_id;
                bool target_movable = db_mapper.EndEffectors.at( target_ee ).movable;
                
                ++removed_transitions;
                if(!source_movable && !target_movable && source_ec == 2)
                    db_writer_eraser.deleteGraspTransition(source_id,target_id);
                else if(!source_movable && target_movable && is_source_sideC && source_ec == 1)
                    db_writer_eraser.deleteGraspTransition(source_id,target_id);
                else
                    --removed_transitions;
            }
        }
        
        db_writer_eraser.close_global();
        ROS_INFO_STREAM("Deleted " << removed_transitions << " unwanted transitions from the initial " << total_transitions << " transitions in the DB..."); 
    }
    
    #endif
    
    for(int i=0; i<5; i++)
    {
        ros::spinOnce();
        usleep(5000);
    }
    
    return 0;
}
