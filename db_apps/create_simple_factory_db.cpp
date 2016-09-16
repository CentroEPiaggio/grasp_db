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

#define CYLINDER_HEIGHT 0.255
#define DB_POSTFIX "simple_factory_"

#define HOW_MANY_ROT 8 // how many rotations for table grasps
#define HOW_MANY_VAR 8 // how many variations for each of the prehensile grasps
#define TINO_WP_HEIGHT 0.0
#define TABLE_WP_HEIGHT 0.05
#define NAMED_TRANSITIONS 1

#define EE_GRASP_FACTOR 100
#define EPS 0.01 // a small additional height
#define OBJ_GRASP_FACTOR dual_manipulation::shared::OBJ_GRASP_FACTOR

/**
 * Assumptions
 *  - a file named empty.db exists with tables already created but empty
 *  - for each object to be added in the database, a bottom grasp already serialized in the folder "object(OBJECT_ID)", with ID (OBJ_GRASP_FACTOR-1)
 *  - database grasp id rule will be: obj_id*OBJ_GRASP_FACTOR + ee_id*EE_GRASP_FACTOR + grasp_id
 */

// DATA that will be stored in the database

// OBJECTS
std::map<int,std::pair<std::string,std::string>> object_list = {
    {51,{"cylinderA","package://dual_manipulation_grasp_db/object_meshes/cylinder.dae"}},
    {52,{"cylinderB","package://dual_manipulation_grasp_db/object_meshes/cylinder.dae"}},
    {53,{"cylinderC","package://dual_manipulation_grasp_db/object_meshes/cylinder.dae"}}
};
std::map<int,KDL::Frame> object_centers = {
    {51,KDL::Frame::Identity()},
    {52,KDL::Frame::Identity()},
    {53,KDL::Frame::Identity()}
};

// END-EFFECTORS
std::vector<uint> ee_ids = {1,2,3,4}; // to stay as generic as possible
std::vector<std::string> ee_name = {"0_right_hand","1_right_hand","tinoBot1_ee","table"};
std::vector<uint> ee_movable = {1,1,1,0}; // can move
std::vector<uint> ee_prehensile = {1,1,0,0}; // can be actuated during grasping
std::vector<std::string> ee_link = {"0_right_hand_palm_link","1_right_hand_palm_link","tinoBot1_ee",""};
std::vector<std::vector<std::string>> ee_prehension_joints = {
    {"0_right_hand_synergy_joint"},
    {"1_right_hand_synergy_joint"},
    {},
    {}
};
std::map<int,std::vector<int>> reachability={
    {1,{1,2}}, // kuka1
    {2,{4,5}}, // kuka2
    {3,{2,3,4}}, // tinoBot
    {4,{1,5}}, // table
};

// NON-PREHENSILE END-EFFECTORS
std::vector<std::vector<KDL::Frame>> ee_nonprehensile_grasps = {
    {},
    {},
    {KDL::Frame(KDL::Vector(0.0,0.0,-1*CYLINDER_HEIGHT/2.0-2.0*EPS)), // tinoBot grasps
        KDL::Frame(KDL::Rotation::RPY(M_PI,0.0,0.0),KDL::Vector(0.0,0.0,CYLINDER_HEIGHT/2.0+2.0*EPS))},
    {KDL::Frame(KDL::Vector(0.0,0.0,-1*CYLINDER_HEIGHT/2.0-EPS)), // table grasps
        KDL::Frame(KDL::Rotation::RPY(M_PI,0.0,0.0),KDL::Vector(0.0,0.0,CYLINDER_HEIGHT/2.0+EPS))},
    {KDL::Frame(KDL::Vector(0.0,0.0,-1*CYLINDER_HEIGHT/2.0-EPS)), // table2 grasps
        KDL::Frame(KDL::Rotation::RPY(M_PI,0.0,0.0),KDL::Vector(0.0,0.0,CYLINDER_HEIGHT/2.0+EPS))}
};
std::vector<std::vector<std::string>> ee_nonprehensile_grasp_names = {
    {},
    {},
    {"bottom","top"}, // tinoBot grasps
    {"bottom","top"}  // table grasps
};

// WORKSPACES
// this vector and WS_Y_MIN, WS_Y_MAX define the workspaces geometry (only for rectangular workspaces all with the same Y bounds)
std::vector<double> ws_x_max({2.5,2.0,1.5,-1.5,-2.0});
std::vector<double> ws_x_min({2.0,1.5,-1.5,-2.0,-2.5});
std::vector<double> ws_y_max({0.5,0.5,0.5,0.5,0.5});
std::vector<double> ws_y_min({-0.5,-0.5,-0.5,-0.5,-0.5});
// adjacencies monolateral information is enough
std::map<int,std::vector<int>> adjacency = {
    {1,{2}},
    {2,{3}},
    {3,{4}},
    {4,{5}}
};

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
}

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

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Grasp db| -> create simple factory db "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "create_simple_factory_db");
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
    std::string command = "cp " + path + "/empty.db " + path + "/" + new_db_name + "\n";
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
        int obj_id = obj.first;
        std::string obj_name = obj.second.first;
        std::string obj_mesh = obj.second.second;
        KDL::Frame obj_frame = object_centers.at(obj_id);
        db_writer.writeNewObject(obj_id,obj_name,obj_mesh,obj_frame);
        
        // for each end-effector
        for(int i=0; i<ee_ids.size(); i++)
        {
            uint ee_id = ee_ids.at(i);
            
            // TODO: if prehensile, generate all grasps, starting from the exemplary one already serialized
            if(ee_prehensile.at(i))
            {
                // make the first grasp from the example one already saved on disk
                // first, create the entry in the database
                db_writer.writeNewGrasp(OBJ_GRASP_FACTOR-1,obj_id,ee_id,"temporary_grasp");
                db_writer.close_global();
                
                // obtain, via specularity, a top and a bottom grasp
                bool top_bottom = true;
                uint grasp_id = 1;
                specularize_grasps(ee_id,ee_link.at(i),ee_prehension_joints.at(i),{OBJ_GRASP_FACTOR-1},{OBJ_GRASP_FACTOR*obj_id+EE_GRASP_FACTOR*ee_id+grasp_id},{"top_" + ee_name.at(i) + "_" + std::to_string(grasp_id)},top_bottom,new_db_name);
                grasp_id = 1 + HOW_MANY_VAR;
                top_bottom = false;
                specularize_grasps(ee_id,ee_link.at(i),ee_prehension_joints.at(i),{OBJ_GRASP_FACTOR-1},{OBJ_GRASP_FACTOR*obj_id+EE_GRASP_FACTOR*ee_id+grasp_id},{"bottom_" + ee_name.at(i) + "_" + std::to_string(grasp_id)},top_bottom,new_db_name);
                
                // Generate via axial symmetry HOW_MANY_VAR-1 new grasps for current end-effector
                KDL::Vector z_axis(0,0,1);
                KDL::Frame rotFrame_obj(KDL::Frame::Identity());
                symmetricGraspMaker sgm(ee_id,ee_link.at(i),ee_prehension_joints.at(i),new_db_name);
                std::vector<uint> new_grasp_ids(HOW_MANY_VAR - 1);
                
                // rotate top grasp
                grasp_id = 1;
                std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),OBJ_GRASP_FACTOR*obj_id+EE_GRASP_FACTOR*ee_id+grasp_id+1);
                if(!sgm.transform_grasp(obj_id,OBJ_GRASP_FACTOR*obj_id+EE_GRASP_FACTOR*ee_id+grasp_id,"top_" + ee_name.at(i),new_grasp_ids,HOW_MANY_VAR,z_axis,rotFrame_obj))
                    return -1;
                // rotate bottom grasp
                grasp_id = 1 + HOW_MANY_VAR;
                std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),OBJ_GRASP_FACTOR*obj_id+EE_GRASP_FACTOR*ee_id+grasp_id+1);
                if(!sgm.transform_grasp(obj_id,OBJ_GRASP_FACTOR*obj_id+EE_GRASP_FACTOR*ee_id+grasp_id,"bottom_" + ee_name.at(i),new_grasp_ids,HOW_MANY_VAR,z_axis,rotFrame_obj))
                    return -1;
                
                // last, delete the temporary grasp entry
                db_writer.open_global();
                db_writer.deleteGrasp(OBJ_GRASP_FACTOR-1);
            }
            // TODO: if movable, generate all grasps as a support surface (like for a table), using the appropriate link
            else if(ee_movable.at(i))
            {
                db_writer.close_global();
                
                int wp_height = TINO_WP_HEIGHT;
                std::string ee_frame_id = ee_link.at(i);
                
                tableGraspMaker tgm_grasps(new_db_name,HOW_MANY_ROT,ee_id,wp_height,ee_frame_id);
                for(int j=0; j<ee_nonprehensile_grasps.at(i).size(); j++)
                {
                    if(!tgm_grasps.create_table_grasps(obj_id, ee_nonprehensile_grasp_names.at(i).at(j) + "_" + ee_name.at(i), ee_nonprehensile_grasps.at(i).at(j),obj_id*OBJ_GRASP_FACTOR + ee_id*EE_GRASP_FACTOR + 1 + j*HOW_MANY_ROT))
                    {
                        ROS_FATAL_STREAM("Unable to create \'" << ee_nonprehensile_grasp_names.at(i).at(j) + "_" + ee_name.at(i) << "\' grasps!!!");
                        return -1;
                    }
                }
                
                db_writer.open_global();
            }
            // TODO: if table, generate all grasps as a support surface
            else
            {
                db_writer.close_global();
                
                int wp_height = TABLE_WP_HEIGHT;
                std::string ee_frame_id = "world";
                
                tableGraspMaker tgm_grasps(new_db_name,HOW_MANY_ROT,ee_id,wp_height,ee_frame_id);
                for(int j=0; j<ee_nonprehensile_grasps.at(i).size(); j++)
                {
                    if(!tgm_grasps.create_table_grasps(obj_id, ee_nonprehensile_grasp_names.at(i).at(j) + "_" + ee_name.at(i), ee_nonprehensile_grasps.at(i).at(j),obj_id*OBJ_GRASP_FACTOR + ee_id*EE_GRASP_FACTOR + 1 + j*HOW_MANY_ROT))
                    {
                        ROS_FATAL_STREAM("Unable to create \'" << ee_nonprehensile_grasp_names.at(i).at(j) + "_" + ee_name.at(i) << "\' grasps!!!");
                        return -1;
                    }
                }
                
                db_writer.open_global();
            }
        }
    }
    
    db_writer.close_global();

    // Make all transitions based on names
    // // THIS COULD BE DONE WITH THE GEOMETRIC FILTER
#if NAMED_TRANSITIONS>0
    std::vector<std::string> prefixes({"bottom","top"}); //,"sidehigh","sidelow"});
    std::map<std::string,std::vector<std::string>> correspondences;
    correspondences["bottom"]    = {"top"}; //,      "sidehigh"};
    // correspondences["sidelow"]  = {"top",      "sidehigh"};
    correspondences["top"]       = {"bottom"}; //,   "sidelow"};
    // correspondences["sidehigh"] = {"bottom",   "sidelow"};
    
    namedAutomaticTransitions nat( prefixes, correspondences, new_db_name );
    
    bool write_ok = nat.write_transitions();
    if(!write_ok)
    {
        ROS_FATAL_STREAM("Unable to write transitions!!!");
        return -1;
    }
#endif
    
    for(int i=0; i<5; i++)
    {
        ros::spinOnce();
        usleep(5000);
    }

    return 0;
}
