#include <ros/ros.h>
#include <grasp_creation_utilities/create_db_utils.h>
#include <grasp_creation_utilities/specular_grasp_maker.h>

bool specularize_grasps(endeffector_id new_ee_id,std::string new_link_name,std::vector<std::string> new_joint_names,std::vector<grasp_id> specularized_grasps,std::vector<grasp_id> new_grasp_ids,std::vector<constraint_id> new_grasp_ecs,std::vector<std::string> specularized_grasp_names,bool top_bottom,std::string db_name)
{
    databaseMapper db_mapper( db_name );
    
    assert( specularized_grasps.size() == specularized_grasp_names.size() );
    assert( specularized_grasps.size() == new_grasp_ids.size() );
    
    specularGraspMaker sgm( new_ee_id, new_link_name, new_joint_names, db_name );
    
    for(int i=0; i < specularized_grasps.size(); i++)
    {
        grasp_id grasp_id_ = specularized_grasps.at(i);
        grasp_id new_grasp_id = new_grasp_ids.at(i);
        constraint_id new_grasp_ec = new_grasp_ecs.at(i);
        std::string new_grasp_name = specularized_grasp_names.at(i);
        
        object_id obj_id;
        endeffector_id ee_id;
        std::string grasp_name;
        
        obj_id = db_mapper.Grasps.at(grasp_id_).obj_id;
        ee_id = db_mapper.Grasps.at(grasp_id_).ee_id;
        grasp_name = db_mapper.Grasps.at(grasp_id_).name;
        
        ROS_INFO_STREAM("Converting grasp " << grasp_name << " > " << new_grasp_name << " (" << (i+1) << " out of " << specularized_grasps.size() << ")");
        
        bool transform_ok;
        transform_ok = sgm.transform_grasp( obj_id, grasp_id_, new_grasp_name, top_bottom, new_grasp_id, new_grasp_ec );
        
        if(!transform_ok)
        {
            ROS_FATAL_STREAM("Stopped transformation at grasp " << grasp_name << " > " << new_grasp_name << "!!!");
            return false;
        }
    }
    return true;
}

int writeGlobalDatabaseInformation(databaseWriter& db_writer, std::vector<double>& ws_x_min, std::vector<double>& ws_x_max, std::vector<double>& ws_y_min, std::vector<double>& ws_y_max, std::vector<double>& ws_z_min, std::vector<double>& ws_z_max, std::map<workspace_id,std::vector<workspace_id>>& adjacency, std::vector<endeffector_id>& ee_ids, std::vector<std::string>& ee_name, std::vector<bool>& ee_movable, std::vector<bool>& ee_prehensile, std::vector<std::string>& ee_link, std::vector<std::vector<std::string>>& ee_prehension_joints, std::map<endeffector_id,std::vector<workspace_id>>& reachability, std::vector<constraint_id>& ec_ids, std::vector<std::string>& ec_name, std::map<constraint_id,std::vector<workspace_id>>& ec_reachability, std::map<constraint_id,std::vector<constraint_id>>& ec_adjacency)
{
    // write ws information
    for(int i=1; i<ws_x_max.size()+1; i++)
    {
        double x_side = (ws_x_max.at(i-1) - ws_x_min.at(i-1))/2.0;
        double y_side = (ws_y_max.at(i-1) - ws_y_min.at(i-1))/2.0;
        std::vector<std::pair<double,double>> polygon({{-x_side, y_side},{x_side, y_side},{x_side, -y_side},{-x_side, -y_side}});
        std::pair<double,double> height_min_mx({0.0, ws_z_max.at(i-1) - ws_z_min.at(i-1)});
        
        KDL::Frame centroid(KDL::Vector((ws_x_min.at(i-1)+ws_x_max.at(i-1))/2,(ws_y_min.at(i-1)+ws_y_max.at(i-1))/2, ws_z_min.at(i-1)));
        if(db_writer.writeNewWorkspace(i,"ws" + std::to_string(i), polygon, height_min_mx, centroid) < 0)
            return -1;
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
}
