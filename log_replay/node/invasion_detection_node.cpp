#include "InvasionSim.h"

#ifdef UBUNTU_VERSION_1804
#include <yaml-cpp/exceptions.h>
YAML::BadConversion badconversion;
#endif

int main(int argc, char** argv) {
    using namespace rock::log_replay;

    ros::init(argc, argv, "invasion_detection");

    auto invasion_detect = InvasionSim();

    auto thread = std::thread([&]() {
        invasion_detect.init();
        invasion_detect.run();
    });

    ros::spin();
}
