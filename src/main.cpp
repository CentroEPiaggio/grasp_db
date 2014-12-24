#include <iostream>
#include <ros/ros.h>
#include <sqlite3.h> 
#include <ros/package.h>

static int callback(void *NotUsed, int argc, char **argv, char **azColName)
{
    int i;
    for(i=0; i<argc; i++)
    {
        std::cout<<azColName[i]<<" = "<<(argv[i] ? argv[i] : "NULL")<<std::endl;  
    }
    return 0;
}

int main(int argc, char* argv[])
{
    std::string path = ros::package::getPath("dual_manipulation_grasp_DB");
    ros::init(argc,argv,"database_manager");
    
    sqlite3 *db;
    int rc;
    /* Open database */
    rc = sqlite3_open(path.append("/test.db").c_str(), &db);
    if( rc ){
        std::cout<< "Can't open database" << sqlite3_errmsg(db)<<std::endl;
        return 0;
    }else{
        std::cout<< "Opened database successfully" <<std::endl;
    }

    const char *sql="SELECT name FROM sqlite_master WHERE type='table';";
    char* errMsg;
    rc= sqlite3_exec(db,sql,callback,0,&errMsg);
    if( rc!=SQLITE_OK )
    {
        std::cout<< "SQL error: "<<errMsg<<std::endl;
        sqlite3_free(errMsg);
    }
    sqlite3_close(db);

    return 0;
}