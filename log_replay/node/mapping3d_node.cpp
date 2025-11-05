//
// Created by gaoyuan on 2024/7/22.
//
#include "Mapping3dSim.h"

#ifdef UBUNTU_VERSION_1804
#include <yaml-cpp/exceptions.h>
YAML::BadConversion badconversion;
#endif

int main(int argc, char** argv) {
    using namespace rock::log_replay;

    ros::init(argc, argv, "mapping3d");

    auto mapper_sim = Mapping3dSim();

    auto thread = std::thread([&]() {
        mapper_sim.init();
        mapper_sim.run();
    });

    ros::spin();
}
