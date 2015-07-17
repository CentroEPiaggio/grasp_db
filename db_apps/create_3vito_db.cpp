#include "ros/ros.h"
#include "ros/package.h"
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/databasewriter.h>
#include <ctime>

#define OBJ_GRASP_FACTOR 1000
#define NUM_KUKAS 6
#define NUM_WORKSPACES NUM_KUKAS+1
#define GRASPS_OFFSET 200
#define OBJECT_ID 10

int add_vitos_in_cylinder_db(std::string db_name, int num_vito);

std::map<int,std::vector<int>> reachability={
    {1,{1,2}},
    {2,{1,2,3}},
    {3,{2,3,4}},
    {4,{3,4,5}},
    {5,{4,5,6}},
    {6,{5,6,7}}
};

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Grasp db| -> create 6 kukas with alternate left right hands db "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "create_6kuka_db");
    /* Assumptions
    *  - a file named empty.db exists with tables already created but empty
    *  - a file named cylinder_grasps.db exists with right_hand grasps and end-effectors:
    *    - EE id = 1 (left)
    *    - EE id = 2 (right)
    *    - EE id = 3 (table)
    */
    // get current date/time to use in the naming of the full DB
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [15];
    std::time(&rawtime);
    timeinfo = std::localtime(&rawtime);
    std::strftime(buffer,15,"%Y%m%d_%H%M",timeinfo);
    // std::cout << buffer << std::endl;
    const int object_id = OBJECT_ID;

    // copy empty.db to a new full_{NOW}.db
    std::string path = ros::package::getPath("dual_manipulation_grasp_db");
    std::string new_db_name("full_" + std::string(buffer,15));
    std::string command = "cp " + path + "/empty.db " + path + "/" + new_db_name + "\n";
    system(command.c_str());

    databaseWriter db_writer(new_db_name);
    databaseMapper db_cylinder("cylinder_grasps.db");
    //Workspaces
    for (int i=1;i<NUM_WORKSPACES+1;i++)
    {
        db_writer.writeNewWorkspace(i,"w"+std::to_string(i));
    }
    //Object
    db_writer.writeNewObject(object_id,"cylinder","../../grasp_db/object_meshes/cylinder.dae");
    //EndEffectors
    /* left -> 1,3,5
     * right -> 2,4,6
     */
    for (int i=1;i<NUM_KUKAS/2+1;i++)
    {
        db_writer.writeNewEndEffectors((i-1)*2+1,"left"+std::to_string(i),true);
        db_writer.writeNewEndEffectors(i*2,"right"+std::to_string(i),true);
    }
    db_writer.writeNewEndEffectors(NUM_KUKAS+1,"table",false);
    //Geometry
    for (int i=1;i<NUM_WORKSPACES+1;i++)
    {
        std::string temp_geometry;
        temp_geometry.append(std::to_string(0));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(i*1));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(1.3));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(i*1));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(1.3));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(i*1+1));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(0));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(i*1+1));
        temp_geometry.append(" ");
        db_writer.writeNewGeometry(i,temp_geometry);
    }
    //Reachability
    /* 1 -> 1,2
     * 3 ->   2,3
     * 5 ->       4,5
     * 
     * 2 -> 1,2,3
     * 4 ->     3,4
     * 6 ->       4,5,6
     */
    
    for (int i=1;i<NUM_KUKAS+1;i++)
    {
        for (auto reach:reachability[i])
            db_writer.writeNewReachability(i,reach);
    }
    for (int i=1;i<NUM_WORKSPACES+1;i++)
        db_writer.writeNewReachability(NUM_KUKAS+1,i);
    //Adjacency
    for (int i=1;i<NUM_WORKSPACES+1;i++)
    {
        db_writer.writeNewAdjacency(i,i+1);
    }
    // Grasps - copy all and do nothing about it
    for (auto grasp:db_cylinder.Grasps)
    {
        db_writer.writeNewGrasp(grasp.first,std::get<0>(grasp.second),std::get<1>(grasp.second),std::get<2>(grasp.second));
    }
    std::cout<<__LINE__<<std::endl;
    // call an externally implemented function to do the rest of the job
    int ret;
    ret = add_vitos_in_cylinder_db(new_db_name, NUM_KUKAS/2);

    if(ret<0)
        std::cout << "Something wrong happened inside \'add_vitos_in_cylinder_db\' function!!!" << std::endl;

    return 0;
}