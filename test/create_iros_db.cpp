#include "ros/ros.h"
#include <kdl/frames.hpp>
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/databasewriter.h>
#include "table_grasp_maker.h"

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Grasp db| -> create iros db "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "create_iros_db");

    /* Assumptions
    - workspaces, geometry, adjacencies, reachability
    - 1 object: Cylinder
    - 3 end effectors: table leftarm rightarm
    - hand grasps alreay serialized
    - selected hand grasps in the database (TODO write from here)
      currently the hand grasps are one for each hand!!!!!
    */
    
    tableGraspMaker table_grasps("top-top.db");
    KDL::Frame obj_ee(KDL::Vector(0.0,0.0,0.16));
    table_grasps.create_table_grasps(1,"Bottom",obj_ee);
    
    databaseMapper mapper("top-top.db");
    endeffector_id left,right;
    for (auto ee : mapper.EndEffectors)
    {
        if (std::get<0>(ee.second)=="left_hand")
            left=ee.first;
        if (std::get<0>(ee.second)=="right_hand")
            right=ee.first;
    }
    grasp_id left_top_grasp, right_top_grasp;
    for (auto grasp:mapper.Grasps)
    {
        if (std::get<1>(grasp.second)==left)
            left_top_grasp=grasp.first;
        if (std::get<1>(grasp.second)==right)
            right_top_grasp=grasp.first;
    }
    databaseWriter writer("top-top.db");
    
    for (auto grasp:mapper.Grasps)
    {
        if (grasp.first!=left_top_grasp && grasp.first!=right_top_grasp)
        {
            writer.writeNewTransition(left_top_grasp,grasp.first);
            writer.writeNewTransition(right_top_grasp,grasp.first);
        }
    }
    ros::spinOnce();

    return 0;
}