#include "ros/ros.h"

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
    */
    
    
    
    ros::spinOnce();

    return 0;
}