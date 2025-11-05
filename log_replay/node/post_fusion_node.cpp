#include "PostFusionSim.h"

#ifdef UBUNTU_VERSION_1804
#include <yaml-cpp/exceptions.h>
YAML::BadConversion badconversion;
#endif

int main(int argc, char** argv) {
    using namespace rock::log_replay;

    ros::init(argc, argv, "post_fusion");

    auto post_fusion = PostFusionSim();

    auto thread = std::thread([&]() {
        post_fusion.init();
        post_fusion.run();
    });

    ros::spin();
}
