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
#include <algorithm>

#define BOX_SIDE_A 0.175 // length along x-axis
#define BOX_SIDE_B 0.112 // length along y-axis
#define BOX_SIDE_C 0.046 // length along z-axis
#define DB_POSTFIX "box_soma_"
#define EMPTY_DB "empty_soma"

#define TINO_WP_HEIGHT 0.0
#define TABLE_WP_HEIGHT 0.05
#define EPS 0.01 // a small additional height
#define NAMED_TRANSITIONS 1
// constraint id's
#define NO_CONSTRAINT_ID 1
#define EDGE_CONSTRAINT_ID 2
#define WALL_CONSTRAINT_ID 3

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
    {   KDL::Frame(KDL::Vector(0.0,0.0,-1*BOX_SIDE_C/2.0-EPS)), // table
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
    {   {NO_CONSTRAINT_ID,EDGE_CONSTRAINT_ID},{NO_CONSTRAINT_ID},{NO_CONSTRAINT_ID},
        {NO_CONSTRAINT_ID,EDGE_CONSTRAINT_ID},{NO_CONSTRAINT_ID},{NO_CONSTRAINT_ID} }  // table
};

// WORKSPACES
// this vector and WS_Y_MIN, WS_Y_MAX define the workspaces geometry (only for rectangular workspaces all with the same Y bounds)
std::vector<double> ws_x_min({-1.7,-1.7,-1.7});
std::vector<double> ws_x_max({0.0,0.0,0.0});
std::vector<double> ws_y_min({-0.6,-0.15,0.15});
std::vector<double> ws_y_max({-0.15,0.15,0.6});
// adjacencies monolateral information is enough
std::map<workspace_id,std::vector<workspace_id>> adjacency = {
    {1,{2}},
    {2,{3}}
};

// ENVIRONMENTAL CONSTRAINTS
std::vector<constraint_id> ec_ids = {NO_CONSTRAINT_ID,EDGE_CONSTRAINT_ID,WALL_CONSTRAINT_ID}; // to stay as generic as possible
std::vector<std::string> ec_name = {"None","Edge","Wall"};
std::map<constraint_id,std::vector<workspace_id>> ec_reachability={
    {NO_CONSTRAINT_ID,      {1,2,3}},
    {EDGE_CONSTRAINT_ID,    {1,2,3}},
    {WALL_CONSTRAINT_ID,    {2}}
};
// bi-lateral information needed
std::map<constraint_id,std::vector<constraint_id>> ec_adjacency = {
    {NO_CONSTRAINT_ID,      {EDGE_CONSTRAINT_ID,WALL_CONSTRAINT_ID}},
    {EDGE_CONSTRAINT_ID,    {NO_CONSTRAINT_ID}},
    {WALL_CONSTRAINT_ID,    {NO_CONSTRAINT_ID}}
};

inline grasp_id computeGraspId(object_id obj_id, endeffector_id ee_id, constraint_id ec_id, grasp_id id_offset)
{
    return obj_id*OBJ_GRASP_FACTOR + ee_id*EE_GRASP_FACTOR + ec_id*EC_GRASP_FACTOR + id_offset;
}

std::string convert_to_rectangular_geometry_string(double x_min, double x_max, double y_min, double y_max)
{
    std::string temp_geometry;
    temp_geometry = std::to_string(x_min) + " " + std::to_string(y_max) + " " + std::to_string(x_max) + " " + std::to_string(y_max) + " " + std::to_string(x_max) + " " + std::to_string(y_min) + " " + std::to_string(x_min) + " " + std::to_string(y_min);
    
    return temp_geometry;
}

int writeGlobalDatabaseInformation(databaseWriter& db_writer)
{
    // write ws information
    for(int i=1; i<ws_x_max.size()+1; i++)
    {
        if(db_writer.writeNewWorkspace(i,"ws" + std::to_string(i)) < 0)
            return -1;
        if(db_writer.writeNewGeometry(i,convert_to_rectangular_geometry_string(ws_x_min.at(i-1),ws_x_max.at(i-1),ws_y_min.at(i-1),ws_y_max.at(i-1))) < 0)
            return -2;
    }
    for(auto ws_adj:adjacency)
    {
        int ws_source = ws_adj.first;
        for(auto ws_target:ws_adj.second)
            if(db_writer.writeNewAdjacency(ws_source,ws_target) < 0)
                return -3;
    }
    
    // write end-effector information
    assert(ee_name.size() == ee_link.size());
    assert(ee_name.size() == ee_ids.size());
    assert(ee_name.size() == ee_prehension_joints.size());
    assert(ee_name.size() == ee_prehensile.size());
    assert(ee_name.size() == ee_movable.size());
    for(int i=0; i<ee_name.size(); i++)
    {
        if(db_writer.writeNewEndEffectors(ee_ids.at(i),ee_name.at(i),bool(ee_movable.at(i))) < 0)
            return -4;
    }
    
    // write reachability information
    for(auto r:reachability)
    {
        int ee = r.first;
        for(auto ws:r.second)
            if(db_writer.writeNewReachability(ee,ws) < 0)
                return -5;
    }
    
    // write environmental constraint information
    assert(ec_name.size() == ec_ids.size());
    assert(ec_name.size() == ec_adjacency.size());
    assert(ec_name.size() == ec_reachability.size());
    for(int i=0; i<ec_name.size(); ++i)
    {
        int source = ec_ids.at(i);
        if(db_writer.writeNewEnvironmentConstraint(source,ec_name.at(i)) < 0)
            return -6;
    }
    for(int source:ec_ids)
    {
        for(auto target:ec_adjacency.at(source))
            if(db_writer.writeNewECAdjacency(source,target) < 0)
                return -7;
        for(auto ws:ec_reachability.at(source))
            if(db_writer.writeNewECReachability(source,ws) < 0)
                return -8;
    }
}

bool specularize_grasps(endeffector_id new_ee_id,std::string new_link_name,std::vector<std::string> new_joint_names,std::vector<grasp_id> specularized_grasps,std::vector<grasp_id> new_grasp_ids,std::vector<constraint_id> new_grasp_ecs,std::vector<std::string> specularized_grasp_names,bool top_bottom,std::string db_name)
{
    databaseMapper db_mapper( db_name );
    
    assert( specularized_grasps.size() == specularized_grasp_names.size() );
    assert( specularized_grasps.size() == new_grasp_ids.size() );
    
    specularGraspMaker sgm( new_ee_id, new_link_name, new_joint_names, db_name );
    
    for(int i=0; i < specularized_grasps.size(); i++)
    {
        grasp_id grasp_id_ = specularized_grasps.at(i);
        grasp_id new_grasp_id = new_grasp_ids.at(i);
        constraint_id new_grasp_ec = new_grasp_ecs.at(i);
        std::string new_grasp_name = specularized_grasp_names.at(i);
        
        object_id obj_id;
        endeffector_id ee_id;
        std::string grasp_name;
        
        obj_id = std::get<0>(db_mapper.Grasps.at(grasp_id_));
        ee_id = std::get<1>(db_mapper.Grasps.at(grasp_id_));
        grasp_name = std::get<2>(db_mapper.Grasps.at(grasp_id_));
        
        ROS_INFO_STREAM("Converting grasp " << grasp_name << " > " << new_grasp_name << " (" << (i+1) << " out of " << specularized_grasps.size() << ")");
        
        bool transform_ok;
        transform_ok = sgm.transform_grasp( obj_id, grasp_id_, new_grasp_name, top_bottom, new_grasp_id, new_grasp_ec );
        
        if(!transform_ok)
        {
            ROS_FATAL_STREAM("Stopped transformation at grasp " << grasp_name << " > " << new_grasp_name << "!!!");
            return false;
        }
    }
    return true;
}

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Grasp db| -> create box soma db "<<std::endl;
    std::cout<<std::endl;
    
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
    
    int ret = writeGlobalDatabaseInformation(db_writer);
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
    std::vector<std::string> prefixes({"sideA_bottom","sideA_top","sideB_bottom","sideB_top","sideC_bottom","sideC_top","sideC"});
    std::map<std::string,std::vector<std::string>> correspondences;
    correspondences["sideA_bottom"] = {"sideA_top"};
    correspondences["sideA_top"] = {"sideA_bottom"};
    correspondences["sideB_bottom"] = {"sideB_top"};
    correspondences["sideB_top"] = {"sideB_bottom"};
    correspondences["sideC_bottom"] = {"sideC_bottom"};
    correspondences["sideC_top"] = {"sideC_top"};
    correspondences["sideC"] = {"sideA","sideB"}; // needed for the edge grasp transitions, but will also add the same grasps (that will always fail) from the table...
    
    namedAutomaticTransitions nat( prefixes, correspondences, new_db_name );
    
    bool write_ok = nat.write_transitions();
    if(!write_ok)
    {
        ROS_FATAL_STREAM("Unable to write transitions!!!");
        return -1;
    }
    
    // TODO delete transitions from sideC NO_CONSTRAINT_ID to sideA/sideB, and between top/bottom of sideA/sideB for the table
    // databaseMapper db_mapper(new_db_name);
    // // find stuff
    // // for all wrong transitions
    // db_writer.deleteGraspTransition(source_id,target_id);
    
    #endif
    
    for(int i=0; i<5; i++)
    {
        ros::spinOnce();
        usleep(5000);
    }
    
    return 0;
}
