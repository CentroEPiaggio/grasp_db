#include "ros/ros.h"
#include <kdl/frames.hpp>
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/databasewriter.h>
#include "grasp_creation_utilities/table_grasp_maker.h"
#include "grasp_creation_utilities/specular_grasp_maker.h"
#include "grasp_creation_utilities/symmetric_grasp_maker.h"
#include "grasp_creation_utilities/named_automatic_transitions.h"
#include "grasp_creation_utilities/geometric_automatic_transitions.h"

/* Assumptions
 * - *workspaces, geometry, adjacencies, reachability
 * - 1 object: cylinder (with ID = 1)
 * - 3 end effectors: 1 left_hand 2 right_hand 3 table
 * - only two grasps (with id 1 and 2) are present for the right hand: top, and side_high
 * - make other grasps via rotation of the object
 * - specularize grasps from ee 2 to ee 1
 * - make table grasps
 */

#define SOURCE_EE_ID 2
#define TARGET_EE_ID 1

#define DB_NAME "cylinder.db"
#define OBJECT_ID 1

#define SOURCE_EE_FRAME "right_hand_palm_link"
#define SOURCE_JOINTS {"right_hand_synergy_joint"}
#define TARGET_EE_FRAME "left_hand_palm_link"
#define TARGET_JOINTS {"left_hand_synergy_joint"}
#define EE_GRASP_NR 16
#define DO_SIDE_GRASPS 0
#define NAMED_TRANSITIONS 1

#if DO_SIDE_GRASPS
#define FIRST_FREE_GRASP 3
#else
#define FIRST_FREE_GRASP 2
#endif

#define CYLINDER_HEIGHT 0.255
#define CYLINDER_RADIUS 0.031
#define TABLE_WP_HEIGHT 0.1
#define HOW_MANY_TABLE_ROT 16
#define TABLE_EE_ID 3
// a small additional height
#define EPS 0.001

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Grasp db| -> create cylinder db "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "create_cylinder_db");
    std::string db_name = DB_NAME;

    // generate new grasps from the two which are present in the DB
    {
        KDL::Vector z_axis(0,0,1);
        KDL::Frame rotFrame_obj(KDL::Frame::Identity());
        symmetricGraspMaker sgm(SOURCE_EE_ID,SOURCE_EE_FRAME,SOURCE_JOINTS,db_name);
        std::vector<uint> new_grasp_ids(EE_GRASP_NR - 1);
        
        // rotate top grasp
        std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),FIRST_FREE_GRASP);
        if(!sgm.transform_grasp(OBJECT_ID,1,"top",new_grasp_ids,EE_GRASP_NR,z_axis,rotFrame_obj))
            return -1;
#if DO_SIDE_GRASPS
        // rotate side grasp
        std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),new_grasp_ids.back()+1);
        if(!sgm.transform_grasp(OBJECT_ID,2,"side_high",new_grasp_ids,EE_GRASP_NR,z_axis,rotFrame_obj))
            return -1;
#endif

        // make a single bottom and side_low
        uint bottom_id = new_grasp_ids.back()+1;
        uint last_id = bottom_id;
        KDL::Vector x_axis(1,0,0);
        if(!sgm.transform_grasp(OBJECT_ID,1,"bottom",std::vector<uint>({bottom_id}),2,x_axis,rotFrame_obj))
            return -1;
#if DO_SIDE_GRASPS
        uint side_low_id = new_grasp_ids.back()+2;
        last_id = side_low_id;
        if(!sgm.transform_grasp(OBJECT_ID,2,"side_low",std::vector<uint>({side_low_id}),2,x_axis,rotFrame_obj))
            return -1;
#endif

        // rotate bottom grasp
        std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),last_id+1);
        if(!sgm.transform_grasp(OBJECT_ID,bottom_id,"bottom",new_grasp_ids,EE_GRASP_NR,z_axis,rotFrame_obj))
            return -1;
#if DO_SIDE_GRASPS
        // rotate side grasp
        std::iota(new_grasp_ids.begin(),new_grasp_ids.end(),new_grasp_ids.back()+1);
        if(!sgm.transform_grasp(OBJECT_ID,side_low_id,"side_low",new_grasp_ids,EE_GRASP_NR,z_axis,rotFrame_obj))
            return -1;
#endif
    }

    uint old_ee_id = SOURCE_EE_ID;
    uint new_ee_id = TARGET_EE_ID;
    std::string new_link_name = TARGET_EE_FRAME;
    std::vector<std::string> new_joint_names = TARGET_JOINTS;
    
    // Create specular grasps for all grasps in the DB
    {
        databaseMapper db_mapper( db_name );
        bool top_bottom = false;

        specularGraspMaker sgm( new_ee_id, new_link_name, new_joint_names, db_name );

        for( auto grasp_id: db_mapper.Grasps )
        {
            uint ee_id = std::get<1>(grasp_id.second);

            if( ee_id == old_ee_id )
            {
                uint obj_id = std::get<0>(grasp_id.second);
                std::string grasp_name = std::get<2>(grasp_id.second);
                std::string new_grasp_name = grasp_name + " (specular)";

                ROS_INFO_STREAM("Converting grasp " << grasp_name << " > " << new_grasp_name << " (" << grasp_id.first << " out of " << db_mapper.Grasps.size() << ")");

                if(!sgm.transform_grasp( obj_id, grasp_id.first, new_grasp_name, top_bottom ))
                {
                    ROS_FATAL_STREAM("Stopped transformation at grasp " << grasp_name << " > " << new_grasp_name << "!!!");
                    return -1;
                }
            }
        }
    }

    // create table grasps
    {
        tableGraspMaker table_grasps(db_name,HOW_MANY_TABLE_ROT,TABLE_EE_ID,TABLE_WP_HEIGHT);
        std::vector<KDL::Frame> table_grasp_frames;
        table_grasp_frames.emplace_back(KDL::Frame(KDL::Vector(0.0,0.0,-1*CYLINDER_HEIGHT/2.0-EPS)));
        table_grasp_frames.emplace_back(KDL::Frame(KDL::Rotation::RPY(M_PI,0.0,0.0),KDL::Vector(0.0,0.0,CYLINDER_HEIGHT/2.0+EPS)));
        table_grasp_frames.emplace_back(KDL::Frame(KDL::Rotation::RPY(M_PI/2.0,0.0,0.0),KDL::Vector(0.0,-CYLINDER_RADIUS-EPS,0.0)));
        table_grasp_frames.emplace_back(KDL::Frame(KDL::Rotation::RPY(-M_PI/2.0,0.0,0.0),KDL::Vector(0.0,CYLINDER_RADIUS+EPS,0.0)));
        std::vector<std::string> table_grasp_names({"bottom","top"});
#if DO_SIDE_GRASPS
        table_grasp_names.push_back("table_side_A");
        table_grasp_names.push_back("table_side_B");
#endif
        for(int i=0; i<table_grasp_names.size(); i++)
            if(!table_grasps.create_table_grasps(OBJECT_ID, table_grasp_names.at(i), table_grasp_frames.at(i),0))
            {
                ROS_FATAL_STREAM("Unable to create \'" << table_grasp_names.at(i) << "\' table grasps!!!");
                return -1;
            }
    }

#if NAMED_TRANSITIONS
    // make named transitions
#if DO_SIDE_GRASPS
    std::vector<std::string> prefixes({"bottom","top","side_low","side_high","table_side"});
#else
    std::vector<std::string> prefixes({"bottom","top"});
#endif
    std::map<std::string,std::vector<std::string>> correspondences;
    correspondences["bottom"] = {"top"};
    correspondences["top"] = {"bottom"};
#if DO_SIDE_GRASPS
    correspondences["bottom"].push_back("side_high");
    correspondences["top"].push_back("side_low");
    correspondences["side_low"] = {"top","side_high","table_side"};
    correspondences["side_high"] = {"bottom","side_low","table_side"};
    correspondences["table_side"] = {"side_low","side_high"};
#endif
    
    namedAutomaticTransitions nat( prefixes, correspondences, DB_NAME );
    
    bool write_ok = nat.write_transitions();
    if(!write_ok)
    {
        ROS_FATAL_STREAM("Unable to write transitions!!!");
        return -1;
    }
#else
    // make geometric transitions
    double hand_sphere_size = 0.1;
    GeometricAutomaticTransitions transitioner(db_name,hand_sphere_size);
    if(!transitioner.writeTransitions())
    {
        ROS_FATAL_STREAM("Unable to write transitions!!!");
        return -1;
    }
#endif

  return 0;
}
