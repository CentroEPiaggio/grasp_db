#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"

bool normalizePose(geometry_msgs::Pose& pose)
{
    geometry_msgs::Quaternion& q(pose.orientation);
    double q_norm = std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    q.x = q.x/q_norm;
    q.y = q.y/q_norm;
    q.z = q.z/q_norm;
    q.w = q.w/q_norm;
    
    bool ok = (q_norm < 1.01 && q_norm > 0.99);
    
    if(!ok)
        ROS_WARN_STREAM("Pose not properly normalized, quaternion norm was " << q_norm);
    
    return ok;
}

bool normalizePoses(std::vector< geometry_msgs::Pose >& poses)
{
    bool ok = true;
    for (auto& p:poses)
        ok = ok & normalizePose(p);
    
    return ok;
}