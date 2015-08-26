#include "ros/ros.h"
#include "ros/package.h"
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/databasewriter.h>
#include <ctime>

#define OBJ_GRASP_FACTOR 1000
#define NUM_KUKAS 6
#define NUM_WORKSPACES 12
#define SOURCE_EE_ID 2
#define GRASPS_OFFSET (100*SOURCE_EE_ID)
#define OBJECT_ID 10
#define WS_Y_MAX 0.0
#define WS_Y_MIN -1.3
#define NUM_END_EFFECTORS NUM_KUKAS+2
#define HOW_MANY_HAND_GRASPS 8

int add_vitos_in_cylinder_db(std::string db_name, int num_vito, int how_many_var);

std::map<int,std::vector<int>> reachability={
    {1,{1,2}},
    {2,{2,3,4}},
    {3,{4,5,6}},
    {4,{6,7,8}},            // also 7, but it's a hole: neglect it?
    // there is no 5 in the final urdf {5,{8,9,10}},
    {6,{10,11,12}},         // also 11, but it's a hole: neglect it?
    // table reachability
    {7,{1,2,3,4,5,12}},
    // belt reachability
    {8,{8,9,10}}            // also 9, but it's on the belt: neglect it?
};

// this vector and WS_Y_MIN, WS_Y_MAX define the workspaces geometry (only for rectangular workspaces all with the same Y bounds)
std::vector<double> ws_x_coords({0,0.8,1.2,1.8,2.2,2.8,3.2,3.5,4.2,4.8,5.5,6,7});

// which Kukas do not create
std::vector<int> dont_kuka({5});

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Grasp db| -> create 6 kukas with alternate left right hands db "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "create_6kuka_db");
    /* Assumptions
    *  - a file named empty.db exists with tables already created but empty
    *  - two right_hand grasps (bottom and sidelow) already serialized in the folder "object($OBJECT_ID)", with IDs (GRASPS_OFFSET+1) and (GRASPS_OFFSET+HOW_MANY_HAND_GRASPS+1)
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
    //Workspaces
    for (int i=1;i<NUM_WORKSPACES+1;i++)
    {
        db_writer.writeNewWorkspace(i,"w"+std::to_string(i));
    }
    //Object
    db_writer.writeNewObject(object_id,"cylinder","../../dualmanipulation/grasp_db/object_meshes/cylinder.dae");
    //EndEffectors
    /* left -> 1,3,5
     * right -> 2,4,6
     */
    for (int i=1;i<NUM_KUKAS/2+1;i++)
    {
        if(std::find(dont_kuka.begin(), dont_kuka.end(), (i-1)*2+1) == dont_kuka.end())
            db_writer.writeNewEndEffectors((i-1)*2+1,std::to_string(i-1)+"_left_hand",true);
        if(std::find(dont_kuka.begin(), dont_kuka.end(), i*2) == dont_kuka.end())
            db_writer.writeNewEndEffectors(i*2,std::to_string(i-1)+"_right_hand",true);
    }
    db_writer.writeNewEndEffectors(NUM_KUKAS+1,"table",false);
    db_writer.writeNewEndEffectors(NUM_KUKAS+2,"belt",true);
    //Geometry
    for (int i=0;i<NUM_WORKSPACES;i++)
    {
        std::string temp_geometry;
        temp_geometry.append(std::to_string(ws_x_coords.at(i)));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(WS_Y_MAX));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(ws_x_coords.at(i)));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(WS_Y_MIN));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(ws_x_coords.at(i+1)));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(WS_Y_MIN));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(ws_x_coords.at(i+1)));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(WS_Y_MAX));
        temp_geometry.append(" ");
        db_writer.writeNewGeometry(i+1,temp_geometry);
    }
    //Reachability
    for (int i=1;i<NUM_END_EFFECTORS+1;i++)
    {
        for (auto reach:reachability[i])
            db_writer.writeNewReachability(i,reach);
    }
    //Adjacency
    for (int i=1;i<NUM_WORKSPACES+1;i++)
    {
        db_writer.writeNewAdjacency(i,i+1);
    }
    // Grasps - there have to be only two
    db_writer.writeNewGrasp(GRASPS_OFFSET+1,OBJECT_ID,SOURCE_EE_ID,"bottom");
    db_writer.writeNewGrasp(GRASPS_OFFSET+HOW_MANY_HAND_GRASPS+1,OBJECT_ID,SOURCE_EE_ID,"sidelow");

    // call an externally implemented function to do the rest of the job
    int ret;
    ret = add_vitos_in_cylinder_db(new_db_name, NUM_KUKAS/2,HOW_MANY_HAND_GRASPS);

    if(ret<0)
        std::cout << "Something wrong happened inside \'add_vitos_in_cylinder_db\' function!!!" << std::endl;

    return 0;
}