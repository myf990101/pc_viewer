#include "LogReplay.h"

#ifdef UBUNTU_VERSION_1804
#include <yaml-cpp/exceptions.h>
YAML::BadConversion badconversion;
#endif

int main(int argc, char** argv) {
    using namespace rock::log_replay;

    ros::init(argc, argv, "log_replay");

    auto replay = LogReplay();

    auto thread = std::thread([&]() {
        replay.init();
        replay.run();
    });

    ros::spin();
}
