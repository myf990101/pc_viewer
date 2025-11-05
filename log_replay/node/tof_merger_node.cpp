#include "TofMergerSim.h"

#ifdef UBUNTU_VERSION_1804
#include <yaml-cpp/exceptions.h>
YAML::BadConversion badconversion;
#endif

int main(int argc, char** argv) {
    using namespace rock::log_replay;

    ros::init(argc, argv, "tof_merger");

    auto merger = TofMergerSim();

    bool regression_mode = false;
    if (!ros::param::get("/regression_mode", regression_mode)) {
        ROS_ERROR("Failed to get param 'regression_mode'");
    }

    merger.log_parser()->set_realtime(!regression_mode);
    auto thread = std::thread([&]() {
        merger.init();
        merger.run();
    });

    if (!regression_mode)
        ros::spin();

    if (thread.joinable())
        thread.join();

    ROS_INFO("Node has been shut down.");
    return 0;
}
