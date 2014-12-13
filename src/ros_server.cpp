#include "ros_server.h"


bool ros_server::handleCommands(diagnostic_msgs::SelfTestRequest& req, diagnostic_msgs::SelfTestResponse& res)
{
    return true;
}


ros_server::ros_server()
{
    command_service = nh.advertiseService("database_manager_commands", &ros_server::handleCommands, this);
}


