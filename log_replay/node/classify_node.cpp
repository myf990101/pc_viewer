#include "ClassifySim.h"

#ifdef UBUNTU_VERSION_1804
#include <yaml-cpp/exceptions.h>
YAML::BadConversion badconversion;
#endif

int main(int argc, char** argv) {
    using namespace rock::log_replay;

    ros::init(argc, argv, "classify");

    auto classify = ClassifySim();

    auto thread = std::thread([&]() {
        classify.init();
        classify.run();
    });

    ros::spin();
}
