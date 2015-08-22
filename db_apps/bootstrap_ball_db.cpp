#include "ros/ros.h"
#include <kdl/frames.hpp>
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/databasewriter.h>
#include "grasp_creation_utilities/table_grasp_maker.h"
#include "grasp_creation_utilities/specular_grasp_maker.h"
#include "grasp_creation_utilities/named_automatic_transitions.h"

#define EE_ID 1

#define DB_NAME "ball.db"
#define OBJECT_ID 11

// BEFORE SPECULARIZE; VALIDATE GRASPS
// TO VALIDATE GRASPS; GENERATE TABLE GRASPS AND TRANSITIONS
// AFTER TEST; SPECULARIZE GRASPS AND TRANSITIONS
#define SPECULARIZE false

// ONLY USFUL FOR SPECULARIZATION
#define EE_FRAME "left_hand_palm_link"
#define JOINTS {"left_hand_synergy_joint"}

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Grasp db| -> create ball db "<<std::endl;
    std::cout<<std::endl;
    
    ros::init(argc, argv, "create_ball_db");
    
    /* Assumptions
     *  - workspaces, geometry, adjacencies, reachability
     *  - 1 object: mugD (with ID = 2)
     *  - 3 end effectors: 1 left_hand 2 right_hand 3 table
     *  - specularize grasps from 2 to 1 ee
     */
    
    uint new_ee_id = EE_ID;
    std::string new_link_name = EE_FRAME;
    std::vector<std::string> new_joint_names = JOINTS;
    std::string db_name = DB_NAME;
    
    // JUST CREATE SPECULAR GRASPS
    //if(SPECULARIZE)
    {
        databaseMapper db_mapper( db_name );
        
        // NOTE: STEP 1 > make specular grasps for left hand
        std::vector<uint> specularized_grasps = {1, 2, 3, 4, 5, 6, 7, 8};
        // notice that names are equal, apart from the sign of y!!!
        std::vector<std::string> specularized_grasp_names = {"top_0","top_45","top_90","top_135","top_180","top_225","top_270","top_315"};
        bool top_bottom = false;
        
        assert( specularized_grasps.size() == specularized_grasp_names.size() );
        
        specularGraspMaker sgm( new_ee_id, new_link_name, new_joint_names, db_name );
        
        for( auto grasp_id: db_mapper.Grasps )
        {
            uint ee_id = std::get<1>(grasp_id.second);
            
            if( ee_id == 2 )
            {
                uint obj_id = std::get<0>(grasp_id.second);
                std::string grasp_name = std::get<2>(grasp_id.second);
                
                std::string new_grasp_name = grasp_name + " (specular in xz-plane)";
                
                ROS_INFO_STREAM("Converting grasp " << grasp_name << " > " << new_grasp_name << " (" << grasp_id.first << " out of " << db_mapper.Grasps.size() << ")");
                
                bool transform_ok;
                transform_ok = sgm.transform_grasp( obj_id, grasp_id.first, new_grasp_name, top_bottom );
                
                if(!transform_ok)
                {
                    ROS_FATAL_STREAM("Stopped transformation at grasp " << grasp_name << " > " << new_grasp_name << "!!!");
                    return -1;
                }
            }
        }
        
        // NOTE: STEP 2 > make transitions for specular grasps
        /* THIS IS NOW DONE WITH THE GEOMETRIC FILTER
         *    std::vector<std::string> prefixes({"bottom","x-y-"});
         *    std::map<std::string,std::vector<std::string>> correspondences;
         *    correspondences["bottom"] = {"spec","x-y-_top"};
         *    correspondences["x-y-"] = {"x-y+"};
         * 
         *    namedAutomaticTransitions nat( prefixes, correspondences, DB_NAME );
         * 
         *    bool write_ok = nat.write_transitions();
         *    if(!write_ok)
         *    {
         *      ROS_FATAL_STREAM("Unable to write transitions!!!");
         *      return -1;
    }
    */
    }
    
    // JUST CREATE TABLE GRASPS
    
    //if(!SPECULARIZE)
    {
        // NOTE: STEP 1 > make table grasps
        tableGraspMaker table_grasps(db_name);
        KDL::Frame obj_ee(KDL::Vector(0.0,0.0,-0.06));
        
        if(!table_grasps.create_table_grasps(OBJECT_ID, "bottom", obj_ee))
        {
            ROS_FATAL_STREAM("Unable to create table grasps!!!");
            return -1;
        }
        
        // NOTE: STEP 2 > make transitions for table grasps
         
             
    
    }
    
    std::vector<std::string> prefixes({"bottom","top"});
    std::map<std::string,std::vector<std::string>> correspondences;
    correspondences["bottom"] = {"top"};
    correspondences["top"] = {"bottom"};
    
    namedAutomaticTransitions nat( prefixes, correspondences, DB_NAME );
    
    bool write_ok = nat.write_transitions();
    if(!write_ok)
    {
        ROS_FATAL_STREAM("Unable to write transitions!!!");
        return -1;
    }
    
    // ros::spinOnce();
    
    return 0;
}
