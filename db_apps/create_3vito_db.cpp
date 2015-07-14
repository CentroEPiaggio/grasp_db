#include "ros/ros.h"
#include "ros/package.h"
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/databasewriter.h>
#include <ctime>

#define OBJ_GRASP_FACTOR 1000

int main(int argc, char **argv)
{
    std::cout<<std::endl;
    std::cout<<"|Grasp db| -> create 3 vito robots db "<<std::endl;
    std::cout<<std::endl;

    ros::init(argc, argv, "create_3vito_db");
    /* Assumptions
    *  - a file named empty.db exists with tables already created but empty
    *  - a file named cylinder_grasps.db exists with table grasps:
    *    - EE id = 1 (left)
    *    - Object id = 1
    *    - Grasp id from 1 to 24 with short names
    *
    *    - EE id = 2 (right)
    *    - Object id = 1
    *    - Grasp id from 25 to 48 with short names
    *
    *    - EE id = 3 (table)
    *    - Object id = 1
    *    - Grasp id from 49 to 64 with short names
    *
    *    and with table transitions coherent with grasps
    */
    // get current date/time to use in the naming of the full DB
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [15];
    std::time(&rawtime);
    timeinfo = std::localtime(&rawtime);
    std::strftime(buffer,15,"%Y%m%d_%H%M",timeinfo);
    // std::cout << buffer << std::endl;

    // copy empty.db to a new full_{NOW}.db
    std::string path = ros::package::getPath("dual_manipulation_grasp_db");
    std::string new_db_name("full_" + std::string(buffer,15));
    std::string command = "cp " + path + "/empty.db " + path + "/" + new_db_name + "\n";
    system(command.c_str());

    databaseWriter db_writer(new_db_name);
    databaseMapper db_cylinder("cylinder_grasps.db");
    //Workspaces
    for (int i=1;i<8;i++)
    {
        db_writer.writeNewWorkspace(i,"w"+std::to_string(i));
    }
    //Object
    db_writer.writeNewObject(1,"cylinder","../../grasp_db/object_meshes/cylinder.dae");
    //EndEffectors
    for (int i=1;i<4;i++)
    {
        db_writer.writeNewEndEffectors((i-1)*2+1,"left"+std::to_string(i),true);
        db_writer.writeNewEndEffectors(i*2,"right"+std::to_string(i),true);
    }
    db_writer.writeNewEndEffectors(4*2-1,"table",false);
    //Geometry
    for (int i=1;i<8;i++)
    {
        std::string temp_geometry;
        temp_geometry.append(std::to_string(0));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(i*0.3));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(1.15));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(i*0.3));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(1.15));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(i*0.3+0.3));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(0));
        temp_geometry.append(" ");
        temp_geometry.append(std::to_string(i*0.3+0.3));
        temp_geometry.append(" ");
        db_writer.writeNewGeometry(i,temp_geometry);
    }
    //Reachability
    for (int i=1;i<4;i++)
    {
        db_writer.writeNewReachability((i-1)*2+1,2*i-1);
        db_writer.writeNewReachability((i-1)*2+1,2*i);
        db_writer.writeNewReachability((i-1)*2+1,2*i+1);
        db_writer.writeNewReachability(i*2,2*i-1);
        db_writer.writeNewReachability(i*2,2*i);
        db_writer.writeNewReachability(i*2,2*i+1);
    }
    for (int i=1;i<8;i++)
        db_writer.writeNewReachability(4*2-1,i);
    //Adjacency
    for (int i=1;i<7;i++)
    {
        db_writer.writeNewAdjacency(i,i+1);
    }
    const int object_id = 1;
    //Grasps
    for (int i=1;i<4;i++)
    {
        for (auto grasp:db_cylinder.Grasps)
        {
            if (std::get<1>(grasp.second)==1)
                db_writer.writeNewGrasp(grasp.first+i*200,object_id,(i-1)*2+1,"left"+std::to_string((i-1)*2+1)+std::get<2>(grasp.second));
            else if (std::get<1>(grasp.second)==2)
                db_writer.writeNewGrasp(grasp.first+i*200,object_id,i*2,"right"+std::to_string(i*2)+std::get<2>(grasp.second));
        }
    }
    for (auto grasp:db_cylinder.Grasps)
        if (std::get<1>(grasp.second)==3)
        db_writer.writeNewGrasp(grasp.first,object_id,std::get<1>(grasp.second),std::get<2>(grasp.second));
    //Transitions
    for (auto transition:db_cylinder.Grasp_transitions)
    {
        for (int i=1;i<4;i++)
        {
            for (auto grasp:transition.second)
            {
                if (std::get<1>(db_cylinder.Grasps[transition.first])==1 || std::get<1>(db_cylinder.Grasps[transition.first])==2)
                {
                    if (std::get<1>(db_cylinder.Grasps[grasp])==1 || std::get<1>(db_cylinder.Grasps[grasp])==2)
                    {
                        for (int j=1;j<4;j++)
                        {
                            db_writer.writeNewTransition(transition.first+(i)*200,grasp+(j)*200);
                        }
                    }
                    else
                    {
                        //some EE to Table
                        db_writer.writeNewTransition(transition.first+(i)*200,grasp);
                    }
                }
                else
                {
                    //Table to some EE
                    db_writer.writeNewTransition(transition.first,grasp+(i)*200);
                }
            }
        }
    }
    // this is not working, so doing it manually
    // copy the full database with no timestamp
    // command = "cp " + path + "/" + new_db_name + " " + path + "/full";
    // std::cout << command << std::endl;
    // system(command.c_str());
    return 0;
}