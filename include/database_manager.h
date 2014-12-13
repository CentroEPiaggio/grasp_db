#ifndef DATABASE_MANAGER_H
#define DATABASE_MANAGER_H
#include <string>
#include <vector>
#include <dual_manipulation_shared/grasp.h>

class database_manager
{
public:
    Grasp getGrasp(std::string graspId);
    std::vector<std::string> getGraspIds(std::string objectId);
private:
    void load_database();
    void save_database();
    
};

#endif // DATABASE_MANAGER_H
