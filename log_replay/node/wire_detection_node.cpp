#include "WireDetectionSim.h"

#ifdef UBUNTU_VERSION_1804
#include <yaml-cpp/exceptions.h>
YAML::BadConversion badconversion;
#endif



int main(int argc, char** argv) {
    using namespace rock::log_replay;

    ros::init(argc, argv, "wire_detection");

    auto wire_detect = WireDetectionSim();

    auto thread = std::thread([&]() {
        wire_detect.init();
        wire_detect.run();
    });

    ros::spin();
}
