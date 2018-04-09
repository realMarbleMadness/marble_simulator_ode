//
// Created by yanda on 4/8/18.
//

#include "environment.h"
using namespace rapidjson;

void SETUP::init(std::string setup_path) {
    FILE* fp = fopen(setup_path.c_str(), "rb"); // non-Windows use "r"
    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    Document document;
    document.ParseStream(is);
    fclose(fp);
    const Value& obs = document["obstacles"];
    this->n_obstacles = document["n_obstacles"].GetInt();
    this->the_obstacles.reserve((unsigned)this->n_obstacles);
    for (auto& o : obs.GetArray()){
        double w = o["width"].GetDouble();
        double h = o["height"].GetDouble();
        double x = o["x"].GetDouble();
        double y = o["y"].GetDouble();
        double rot = o["rotation"].GetDouble();
        RECT r = {{w, 0.01, h}, {x, 0, y}, rot};
        this->obstacles.push_back(r);
    }

}