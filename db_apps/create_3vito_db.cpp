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
double ws_z_min = 0.06;
double ws_z_max = 0.64;

// which Kukas do not create
std::vector<int> dont_kuka({5});

// ENVIRONMENTAL CONSTRAINTS
#define NO_CONSTRAINT_ID 1
std::vector<constraint_id> ec_ids = {NO_CONSTRAINT_ID}; // to stay as generic as possible
std::vector<std::string> ec_name = {"None"};
std::map<constraint_id,std::vector<workspace_id>> ec_reachability={
    {NO_CONSTRAINT_ID,      {1,2,3,4,5,6,7,8,9,10,11,12}}
};
// bi-lateral information needed
std::map<constraint_id,std::vector<constraint_id>> ec_adjacency = {
    {NO_CONSTRAINT_ID,      {NO_CONSTRAINT_ID}}
};

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
    //Object
    db_writer.writeNewObject(object_id,"cylinder","package://dual_manipulation_grasp_db/object_meshes/cylinder.dae");
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
    
    // write ws information
    for (int i=0;i<NUM_WORKSPACES;i++)
    {
        double x_side = (ws_x_coords.at(i+1) - ws_x_coords.at(i))/2.0;
        double y_side = (WS_Y_MAX - WS_Y_MIN)/2.0;
        std::vector<std::pair<double,double>> polygon({{-x_side, y_side},{x_side, y_side},{x_side, -y_side},{-x_side, -y_side}});
        std::pair<double,double> height_min_mx({0.0, ws_z_max - ws_z_min});
        
        KDL::Frame centroid(KDL::Vector((ws_x_coords.at(i+1)+ws_x_coords.at(i))/2,(WS_Y_MAX + WS_Y_MIN)/2, ws_z_min));
        if(db_writer.writeNewWorkspace(i+1,"ws" + std::to_string(i+1), polygon, height_min_mx, centroid) < 0)
            return -1;
    }
    //Adjacency
    for (int i=1;i<NUM_WORKSPACES+1;i++)
    {
        db_writer.writeNewAdjacency(i,i+1);
    }
    
    //Reachability
    for (int i=1;i<NUM_END_EFFECTORS+1;i++)
    {
        for (auto reach:reachability[i])
            db_writer.writeNewReachability(i,reach);
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
    
    // Grasps - there have to be only two
    db_writer.writeNewGrasp(GRASPS_OFFSET+1,OBJECT_ID,SOURCE_EE_ID,"bottom_right",NO_CONSTRAINT_ID);
    db_writer.writeNewGrasp(GRASPS_OFFSET+HOW_MANY_HAND_GRASPS+1,OBJECT_ID,SOURCE_EE_ID,"sidelow_right",NO_CONSTRAINT_ID);

    // call an externally implemented function to do the rest of the job
    int ret;
    ret = add_vitos_in_cylinder_db(new_db_name, NUM_KUKAS/2,HOW_MANY_HAND_GRASPS);

    if(ret<0)
        std::cout << "Something wrong happened inside \'add_vitos_in_cylinder_db\' function!!!" << std::endl;

    return 0;
}