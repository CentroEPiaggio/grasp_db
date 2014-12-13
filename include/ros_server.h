#ifndef ROS_SERVER_H
#define ROS_SERVER_H


#include <ros/ros.h>
#include <ros/service.h>

#include <diagnostic_msgs/SelfTest.h>

class ros_server
{
public:
    ros_server();
    ros::ServiceServer command_service;
private:
    bool handleCommands(diagnostic_msgs::SelfTestRequest  &req,
                        diagnostic_msgs::SelfTestResponse &res);
    ros::NodeHandle nh;
};

#endif // ROS_SERVER_H
