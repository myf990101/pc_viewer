//
// Created by pangze on 2023/9/11.
//
#include <yaml-cpp/yaml.h>

#include <cstdint>

#include "detection/InvasionDetector.h"
#include "detection/TopClassify.hpp"
#include "interface/log_file_definition.hpp"
#include "interface/observation_converter.hpp"
#include "log_parser.hpp"
#include "mapping3d/TofMapper.h"
#include "mapprocess/MapWall.hpp"
#include "perceptor.h"
#include "util/Log.h"
#include "util/Timer.h"

extern std::string test_config;
static const auto  WaitForFutureTime = int32_t{50};    //ms

class TestTimestamp {
public:
    static double now() {
        auto duration = std::chrono::steady_clock::now() - instance().actual_start_;
        auto diff     = (double)duration.count() * decltype(duration)::period::num / decltype(duration)::period::den;
        return instance().test_start_ + diff;
    }

    static void reset(double timestamp) {
        instance().test_start_   = timestamp;
        instance().actual_start_ = std::chrono::steady_clock::now();
    }

private:
    static TestTimestamp& instance() {
        static auto timestamp = TestTimestamp{};
        return timestamp;
    }

    TestTimestamp() = default;

    double                                             test_start_;
    std::chrono::time_point<std::chrono::steady_clock> actual_start_;
};

template<typename Init>
auto make_message(RockPerceptorMessageType type, const uint32_t& size, Init init) -> msg_t const* {
    auto message = (msg_t*)rock_perceptor_memory_allocate(sizeof(msg_t) + size);

    message->type = type;
    message->size = static_cast<uint32_t>(size);
    init(message);

    return message;
}

void callback(msg_t const* message) {
    using namespace rock::perceptor;
    switch (message->type) {
        case RockPerceptorMessageType::ClassifyTopTof: {
            auto observation = *(point_cloud_data_t**)message->data;
            rock::perceptor::log::INFO_STREAM("receive message... timestamp = "
                                              << observation->timestamp << ", type = " << observation->type
                                              << ", sensor = " << observation->sensor
                                              << ", point num = " << observation->size);
            break;
        }
        case RockPerceptorMessageType::Invasion: {
            auto invasion_state_msg = *(invasion_t*)message->data;
            int  invasion_state     = (uint8_t)(invasion_state_msg.invasion_state);
            int  move_state         = (uint8_t)(invasion_state_msg.move_state);
            auto observation        = (point_cloud_data_t*)(invasion_state_msg.point_cloud);
            rock::perceptor::log::INFO_STREAM("receive message... timestamp = "
                                              << observation->timestamp << ", type = " << observation->type
                                              << ", sensor = " << observation->sensor << ", point num = "
                                              << observation->size << ", invasion state = " << invasion_state
                                              << ", move state = " << move_state);

            break;
        }
        default:
            break;
    }
}

int main(int argc, char** argv) {
    using namespace rock;
    using namespace rock::log_replay;
    using namespace rock::log_parser;
    using namespace rock::perceptor;
    using namespace rock::perceptor::common;
    using namespace rock::perceptor::detection::invasion;
    using namespace rock::perceptor::detection::topclassify;

    auto conf          = YAML::LoadFile(WORK_SPACE_LOCATION "/config/invasion_detection.yaml");
    auto log_directory = conf["replay"]["log_directory"].as<std::string>();

    test_config = std::string(WORK_SPACE_LOCATION "/config/invasion_detection.yaml");

    auto count      = 0u;
    auto paused     = false;
    auto start_time = 711;
    auto alpha      = 0.5f;

    auto log_parser = rock::log_parser::LogParser(
        {{.type = AvoidCubePath.type, .path = log_directory + AvoidCubePath.path},
         {.type = PointCloudDownSamplePath.type, .path = log_directory + PointCloudDownSamplePath.path},
         {.type = PointCloudSourcePath.type, .path = log_directory + PointCloudSourcePath.path},
         {.type = CommandPath.type, .path = log_directory + CommandPath.path},
         {.type = SensorLogPath.type, .path = log_directory + SensorLogPath.path},
         {.type = ImagePath.type, .path = log_directory + ImagePath.path}});

    log_parser.set_log_time(start_time);

    log_parser.begin();

    auto time_reset = false;

    auto* buffer = new uint8_t[sizeof(perceptor::map::voxel::shared_map_t)];
    auto* buffer2 = new uint8_t[sizeof(rock::perceptor::mapprocess::MapWall::processed_buffer)];
    rock_perceptor_controller_init(callback);
    rock_perceptor_controller_init_shared_memory(SharedMemoryType::SharedMap3d, (void*)buffer, sizeof(map::voxel::shared_map_t));
    rock_perceptor_controller_init_shared_memory(SharedMemoryType::ProcessedGridMap, (void*)buffer2, sizeof(rock::perceptor::mapprocess::MapWall::processed_buffer));
    while (auto data = log_parser.get_data()) {
        auto& raw = data->get();

        if (!time_reset) {
            TestTimestamp::reset(raw.timestamp);
            rock_perceptor_set_chrono(TestTimestamp::now);
            time_reset = true;
        }

        if (!paused) {
            auto wait_time = (raw.timestamp - TestTimestamp::now()) * 1000;

            if (wait_time > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(wait_time)));
            } else {
                TestTimestamp::reset(raw.timestamp);
            }
        }
        if (raw.type == LogDataType::Cmd_Pause && raw.sensor == 1) {
            rock::perceptor::log::DEBUG_STREAM("Cmd_Pause true timestamp: " << raw.timestamp);
            paused = true;
            rock_perceptor_controller_dispatch(make_message(RockPerceptorMessageType::Pause, sizeof(bool),
                                                            [&](msg_t* ptr) { *(bool*)ptr->data = true; }));
            continue;
        }

        if (raw.type == LogDataType::Cmd_Pause && raw.sensor == 0) {
            rock::perceptor::log::DEBUG_STREAM("Cmd_Pause false timestamp: " << raw.timestamp);
            paused = false;
            rock_perceptor_controller_dispatch(make_message(RockPerceptorMessageType::Pause, sizeof(bool),
                                                            [&](msg_t* ptr) { *(bool*)ptr->data = false; }));
            continue;
        }

        if (raw.type == LogDataType::SlamPose && raw.sensor == 0) {
            rock::perceptor::log::DEBUG_STREAM("Receive slam pose: " << raw.timestamp);
            rock_perceptor_controller_dispatch(make_message(RockPerceptorMessageType::SlamPose,
                                                            sizeof(pose_2d_stamped_t), [&](msg_t* ptr) {
                                                                auto& pose     = *(pose_2d_stamped_t*)ptr->data;
                                                                pose.timestamp = raw.timestamp;
                                                                pose.pose.point.x.metre = raw.position_2d.pos.x;
                                                                pose.pose.point.y.metre = raw.position_2d.pos.y;
                                                                pose.pose.theta         = raw.position_2d.pos.theta;
                                                            }));
            continue;
        }

        if (raw.type == LogDataType::AvoidCube) {
            rock::perceptor::log::DEBUG_STREAM("Receive avoidcube: " << raw.timestamp);
            rock_perceptor_controller_dispatch(
                make_message(RockPerceptorMessageType::AvoidCube, sizeof(cube_data_t), [&](msg_t* ptr) {
                    auto& cube     = *(cube_data_t*)ptr->data;
                    cube.timestamp = raw.timestamp;
                    cube.position  = raw.avoid_cube.armPosition;
                    cube.state     = raw.avoid_cube.armworkstatus;
                    cube.size      = 6;
                    std::copy(raw.avoid_cube.seed.begin(), raw.avoid_cube.seed.end(), cube.seed);
                    std::copy(raw.avoid_cube.cube.begin(), raw.avoid_cube.cube.end(), cube.cube);
                }));
            continue;
        }

        if (raw.type == LogDataType::Tof && raw.tof.exposure == Tof::Exposure::Flood
            && raw.sensor == static_cast<int>(Tof::Sensor::Top)) {
            rock::perceptor::log::DEBUG_STREAM("Receive tof: " << raw.timestamp << ", sensor: " << raw.sensor
                                                               << ", type: " << static_cast<int>(raw.tof.exposure));

            rock_perceptor_controller_dispatch(make_message(RockPerceptorMessageType::JudgeInvasionState,
                                                            sizeof(arm_t), [&](msg_t* ptr) {
                                                                auto& switch_flag = *(arm_t*)ptr->data;
                                                                switch_flag.state = true;
                                                            }));

            auto tof         = ObservationConverter::convert_to_tof(raw);
            tof->reserved[0] = 0;
            tof->reserved[1] = tof->size;

            rock_perceptor_controller_dispatch(make_message(
                RockPerceptorMessageType::Tof, sizeof(point_cloud_data_t) + sizeof(point_5d_raw_t) * tof->size,
                [&](msg_t* ptr) {
                    auto& observation      = *(point_cloud_data_t*)(ptr->data);
                    observation            = *tof;
                    observation.type = static_cast<int>(Tof::Exposure::HdrFlood);
                    std::copy(tof->points_5d, tof->points_5d + tof->size,
                              observation.points_5d);
                }));

            ++count;
            continue;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(1000)));
    rock_perceptor_controller_finish();
    return 0;
}
