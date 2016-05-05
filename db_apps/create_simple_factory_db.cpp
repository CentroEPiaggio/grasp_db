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

#define HOW_MANY_ROT 4 // how many rotations for table grasps
#define HOW_MANY_VAR 4 // how many variations for each of the prehensile grasps
#define TINO_WP_HEIGHT 0.0
#define TABLE_WP_HEIGHT 0.05
#define NAMED_TRANSITIONS 1

#define EE_GRASP_FACTOR 100
#define EPS 0.01 // a small additional height

/**
 * Assumptions
 *  - a file named empty.db exists with tables already created but empty
 *  - for each object to be added in the database, a bottom grasp already serialized in the folder "object(OBJECT_ID)", with ID (OBJ_GRASP_FACTOR-1)
 *  - database grasp id rule will be: obj_id*OBJ_GRASP_FACTOR + ee_id*EE_GRASP_FACTOR + grasp_id
 */

// DATA that will be stored in the database
std::map<int,std::pair<std::string,std::string>> object_list = {
    {51,{"cylinderA","package://dual_manipulation_grasp_db/object_meshes/cylinder.dae"}},
    {52,{"cylinderB","package://dual_manipulation_grasp_db/object_meshes/cylinder.dae"}}
};
std::map<int,KDL::Frame> object_centers = {
    {51,KDL::Frame::Identity()},
    {52,KDL::Frame::Identity()}
};

std::vector<uint> ee_ids = {1,2,3,4}; // to stay as generic as possible
std::vector<std::string> ee_name = {"0_right_hand","1_right_hand","tinoBot_ee","table"};
std::vector<uint> ee_movable = {1,1,1,0}; // can move
std::vector<uint> ee_prehensile = {1,1,0,0}; // can be actuated during grasping
std::vector<std::string> ee_link = {"0_right_hand_palm_link","1_right_hand_palm_link","tinoBot_ee",""};
std::vector<std::vector<std::string>> ee_prehension_joints = {
    {"0_right_hand_synergy_joint"},
    {"1_right_hand_synergy_joint"},
    {},
    {}
};

std::vector<std::vector<KDL::Frame>> ee_nonprehensile_grasps = {
    {},
    {},
    {KDL::Frame(KDL::Vector(0.0,0.0,-1*CYLINDER_HEIGHT/2.0-2.0*EPS)), // tinoBot grasps
        KDL::Frame(KDL::Rotation::RPY(M_PI,0.0,0.0),KDL::Vector(0.0,0.0,CYLINDER_HEIGHT/2.0+2.0*EPS))},
        {KDL::Frame(KDL::Vector(0.0,0.0,-1*CYLINDER_HEIGHT/2.0-EPS)), // table grasps
            KDL::Frame(KDL::Rotation::RPY(M_PI,0.0,0.0),KDL::Vector(0.0,0.0,CYLINDER_HEIGHT/2.0+EPS))}
};

std::vector<std::vector<std::string>> ee_nonprehensile_grasp_names = {
    {},
    {},
    {"bottom","top"}, // tinoBot grasps
    {"bottom","top"} // table grasps
};

std::map<int,std::vector<int>> reachability={
    {1,{1,2}}, // kuka1
    {2,{4,5}}, // kuka2
    {3,{2,3,4}}, // tinoBot
    {4,{1,5}} // table
};

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

//     //Generate basic grasps
//     dual_manipulation_shared::grasp_trajectory grasp_msg;
//     std::string grasp_file_name = "object" + std::to_string(object_list.begin()->first) + "/grasp" + std::to_string(OBJ_GRASP_FACTOR-1);
//     
//     if( deserialize_ik( grasp_msg, grasp_file_name ) )
//     {
//         ROS_INFO_STREAM("Deserialization OK!");
//         std::cout << "grasp_msg" << grasp_msg << std::endl;
// 
// // // //         grasp_msg.attObject.object.header.frame_id
//     }
//     else
//     {
//         ROS_ERROR_STREAM("Error in deserialization object!");
//         return -1;
//     }
    
    ros::spin();
//     db_writer.writeNewGrasp(GRASPS_OFFSET+1,OBJECT_ID_1,SOURCE_EE_ID,"bottom");
//     db_writer.writeNewGrasp(GRASPS_OFFSET+HOW_MANY_HAND_GRASPS+1,OBJECT_ID_1,SOURCE_EE_ID,"sidelow");
// 
//     // call an externally implemented function to do the rest of the job
//     int ret;
//     ret = add_vitos_in_cylinder_db(new_db_name, NUM_KUKAS/2,HOW_MANY_HAND_GRASPS);
// 
//     if(ret<0)
//         std::cout << "Something wrong happened inside \'add_vitos_in_cylinder_db\' function!!!" << std::endl;

    return 0;
}

/*
int add_vitos_in_cylinder_db(std::string db_name = DB_NAME, int num_vito = NUM_VITO, int how_many_var = HOW_MANY_VAR)
{ *
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
    std::vector<std::string> table_grasp_names({"bottom","top"});
    for(int i=0; i<table_grasp_names.size(); i++)
        if(!table_grasps.create_table_grasps(OBJECT_ID_1, table_grasp_names.at(i), table_grasp_frames.at(i),1+i*HOW_MANY_ROT))
        {
            ROS_FATAL_STREAM("Unable to create \'" << table_grasp_names.at(i) << "\' table grasps!!!");
            return -1;
        }
        
        // Belt grasps: Bottom and Top, as the table
        tableGraspMaker belt_grasps(db_name,HOW_MANY_ROT,BELT_EE_ID,TINO_WP_HEIGHT,"belt0_p");
    std::vector<KDL::Frame> belt_grasp_frames;
    belt_grasp_frames.emplace_back(KDL::Frame(KDL::Vector(0.0,0.0,-1*CYLINDER_HEIGHT/2.0-2.0*EPS)));
    belt_grasp_frames.emplace_back(KDL::Frame(KDL::Rotation::RPY(M_PI,0.0,0.0),KDL::Vector(0.0,0.0,CYLINDER_HEIGHT/2.0+2.0*EPS)));
    std::vector<std::string> belt_grasp_names({"bottom","top"});
    for(int i=0; i<belt_grasp_names.size(); i++)
        if(!belt_grasps.create_table_grasps(OBJECT_ID_1, belt_grasp_names.at(i), belt_grasp_frames.at(i),BELT_EE_ID*100+1+i*HOW_MANY_ROT))
        {
            ROS_FATAL_STREAM("Unable to create \'" << belt_grasp_names.at(i) << "\' belt grasps!!!");
            return -1;
        }
        
        // PART 0
        // Generate via symmetry how_many_var-1 new grasps for the right hand
            KDL::Vector z_axis(0,0,1);
            KDL::Frame rotFrame_obj(KDL::Frame::Identity());
            symmetricGraspMaker sgm(STARTING_EE_ID,SOURCE_EE_FRAME,SOURCE_JOINTS,db_name);
            std::vector<uint> new_grasp_ids(how_many_var - 1);
            
            // rotate bottom grasp
            std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),SINGLE_HAND_GRASP_LIMIT*STARTING_EE_ID + 2);
            if(!sgm.transform_grasp(OBJECT_ID_1,SINGLE_HAND_GRASP_LIMIT*STARTING_EE_ID + 1,"bottom",new_grasp_ids,how_many_var,z_axis,rotFrame_obj))
                return -1;
            // rotate sidelow grasp
            std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),SINGLE_HAND_GRASP_LIMIT*STARTING_EE_ID + how_many_var + 2);
            if(!sgm.transform_grasp(OBJECT_ID_1,SINGLE_HAND_GRASP_LIMIT*STARTING_EE_ID + how_many_var + 1,"sidelow",new_grasp_ids,how_many_var,z_axis,rotFrame_obj))
                return -1;
        
        // some useful definitions
        std::vector<std::string> top_vector(how_many_var,"top");
        std::vector<std::string> bottom_vector(how_many_var,"bottom");
        std::vector<std::string> sidehigh_vector(how_many_var,"sidehigh");
        std::vector<std::string> sidelow_vector(how_many_var,"sidelow");
        
        // PART 1
        // Specularize right_hand bottom and sidelow into left_hand top and sidehigh
        uint new_ee_id = 1;
        uint old_ee_id = STARTING_EE_ID;
        std::string new_link_name = "left_hand_palm_link";
        std::vector<std::string> new_joint_names = {"left_hand_synergy_joint"};
        std::vector<uint> specularized_grasps(how_many_var*2);
        std::iota(specularized_grasps.begin(),specularized_grasps.end(),SINGLE_HAND_GRASP_LIMIT*old_ee_id + 1);
        
        std::vector<std::string> specularized_grasp_names;
        specularized_grasp_names.clear();
        specularized_grasp_names.insert(specularized_grasp_names.end(),top_vector.begin(),top_vector.end());
        specularized_grasp_names.insert(specularized_grasp_names.end(),sidehigh_vector.begin(),sidehigh_vector.end());
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
        specularized_grasp_names.insert(specularized_grasp_names.end(),bottom_vector.begin(),bottom_vector.end());
        specularized_grasp_names.insert(specularized_grasp_names.end(),sidelow_vector.begin(),sidelow_vector.end());
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
        new_link_name = "right_hand_palm_link";
        new_joint_names=std::vector<std::string>({"right_hand_synergy_joint"});
        std::iota(specularized_grasps.begin(),specularized_grasps.end(),SINGLE_HAND_GRASP_LIMIT*old_ee_id + 1);
        specularized_grasp_names.clear();
        specularized_grasp_names.insert(specularized_grasp_names.end(),top_vector.begin(),top_vector.end());
        specularized_grasp_names.insert(specularized_grasp_names.end(),sidehigh_vector.begin(),sidehigh_vector.end());
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
        specularized_grasp_names.clear();
        specularized_grasp_names.insert(specularized_grasp_names.end(),bottom_vector.begin(),bottom_vector.end());
        specularized_grasp_names.insert(specularized_grasp_names.end(),sidelow_vector.begin(),sidelow_vector.end());
        specularized_grasp_names.insert(specularized_grasp_names.end(),top_vector.begin(),top_vector.end());
        specularized_grasp_names.insert(specularized_grasp_names.end(),sidehigh_vector.begin(),sidehigh_vector.end());
        top_bottom = false;
        specularized_grasps.clear();
        specularized_grasps.resize(specularized_grasp_names.size());
        new_grasp_ids.clear();
        new_grasp_ids.resize(specularized_grasp_names.size());
        
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
            new_link_name =  left_right + "_hand_palm_link";
            new_joint_names=std::vector<std::string>({left_right + "_hand_synergy_joint"});
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
            std::vector<std::string> prefixes({"bottom","top","sidehigh","sidelow"});
            std::map<std::string,std::vector<std::string>> correspondences;
            correspondences["bottom"]    = {"top",      "sidehigh"};
            correspondences["sidelow"]  = {"top",      "sidehigh"};
            correspondences["top"]       = {"bottom",   "sidelow"};
            correspondences["sidehigh"] = {"bottom",   "sidelow"};
            
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
*/
