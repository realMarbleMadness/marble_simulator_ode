
//
// Created by yanda on 3/19/18.
//
#include "utils.h"

void grab_params(ros::NodeHandle &n, const std::string& key, std::string& s) {
    if (n.getParam(key, s))
    {
        ROS_INFO("Got param: %s", s.c_str());
    }
    else
    {
        ROS_ERROR("Failed to get param '%s", key.c_str());
    }
}

void grab_params(ros::NodeHandle &n, const std::string& key, float& f) {
    if (n.getParam(key, f))
    {
        ROS_INFO("Got param: %f", f);
    }
    else
    {
        ROS_ERROR("Failed to get param '%s", key.c_str());
    }
}
