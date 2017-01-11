#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/databasewriter.h>

bool specularize_grasps(endeffector_id new_ee_id,std::string new_link_name,std::vector<std::string> new_joint_names,std::vector<grasp_id> specularized_grasps,std::vector<grasp_id> new_grasp_ids,std::vector<constraint_id> new_grasp_ecs,std::vector<std::string> specularized_grasp_names,bool top_bottom,std::string db_name);

int writeGlobalDatabaseInformation(databaseWriter& db_writer, std::vector<double>& ws_x_min, std::vector<double>& ws_x_max, std::vector<double>& ws_y_min, std::vector<double>& ws_y_max, std::vector<double>& ws_z_min, std::vector<double>& ws_z_max, std::map<workspace_id,std::vector<workspace_id>>& adjacency, std::vector<endeffector_id>& ee_ids, std::vector<std::string>& ee_name, std::vector<bool>& ee_movable, std::vector<bool>& ee_prehensile, std::vector<std::string>& ee_link, std::vector<std::vector<std::string>>& ee_prehension_joints, std::map<endeffector_id,std::vector<workspace_id>>& reachability, std::vector<constraint_id>& ec_ids, std::vector<std::string>& ec_name, std::map<constraint_id,std::vector<workspace_id>>& ec_reachability, std::map<constraint_id,std::vector<constraint_id>>& ec_adjacency);

