//
// Created by yanda on 3/19/18.
//

#ifndef MARBLE_SIMULATOR_ODE_UTILS_H
#define MARBLE_SIMULATOR_ODE_UTILS_H

#include <ros/ros.h>

void grab_params(ros::NodeHandle &n, const std::string& key, std::string& s);
void grab_params(ros::NodeHandle &n, const std::string& key, float& f);

#endif //MARBLE_SIMULATOR_ODE_UTILS_H
