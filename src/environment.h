//
// Created by yanda on 4/8/18.
//

#ifndef MARBLE_SIMULATOR_ODE_ENVIRONMENT_H
#define MARBLE_SIMULATOR_ODE_ENVIRONMENT_H

#include <ode/common.h>
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include <cstdio>
#include <vector>

// used to allocate space for obstacles
typedef struct rect
{
    double dimensions[3];
    double pos[3];
    double angle;
} RECT;

class SETUP
{
public:
    // these are static ODE objects: just geometries
    dGeomID the_wall;
    std::vector<RECT> obstacles;
    std::vector<dGeomID> the_obstacles;
    int n_obstacles;
    void init(std::string setup_path);
};


#endif //MARBLE_SIMULATOR_ODE_ENVIRONMENT_H
