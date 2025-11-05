#include <yaml-cpp/yaml.h>

// #include "global_definition.h"
#include "interface/log_file_definition.hpp"
#include "interface/observation_converter.hpp"
#include "log_parser.hpp"
#include "mapping3d/Option.h"
#include "mapping3d/TofMapper.h"
#include "util/Log.h"

int main(int argc, char** argv) {
    using namespace rock;
    using namespace rock::perceptor;
    using namespace rock::log_replay;
    using namespace rock::perceptor::map;
    using namespace rock::perceptor::common;

    using mapper_t = mapping3d::TofMapper;

    auto count                      = 0u;
    auto valid_reset                = true;
    auto paused                     = false;
    auto last_motion_timestamp      = double{-1};
    auto last_observation_timestamp = double{-1};

    auto conf          = YAML::LoadFile(WORK_SPACE_LOCATION "/config/mapping3d.yaml");
    auto log_directory = conf["replay"]["log_directory"].as<std::string>();
    auto tof_type      = conf["map"]["tof_type"].as<int32_t>();
    auto use_tof       = std::array<uint64_t, 2>{};

    if (log_directory.empty()) {
        log::ERROR("No valid log directory found, exit.");
        exit(0);
    }

    auto mapper = std::make_unique<mapping3d::TofMapper>();
//    mapper->init_from_file(WORK_SPACE_LOCATION"/config/mapping3d.yaml");
    mapper->init(tof_type, log_directory);

    if (!mapper->is_init()) {
        log::WARN("Fail to initialize system system");
        return -1;
    }

    mapping3d::option.save_map_file = conf["map"]["save_map_directory"].as<std::string>();

    auto log_parser = rock::log_parser::LogParser(
        {{.type = PointCloudDownSamplePath.type, .path = log_directory + PointCloudDownSamplePath.path},
         {.type = CommandPath.type, .path = log_directory + CommandPath.path},
         {.type = SensorLogPath.type, .path = log_directory + SensorLogPath.path}});

    auto* buffer = new uint8_t[sizeof(perceptor::map::voxel::shared_map_t)];
    mapper_t::init_shared_memory(SharedMemoryType::SharedMap3d, buffer, sizeof(perceptor::map::voxel::shared_map_t));

    for (auto& raw : log_parser) {
        if (raw.type == rock::log_parser::LogDataType::Cmd_Pause && raw.sensor == 1) {
            log::DEBUG("Cmd_Pause true timestamp: %f", raw.timestamp);
            last_motion_timestamp = last_observation_timestamp = raw.timestamp;
            mapper->pause();
            paused = true;
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_Pause && raw.sensor == 0) {
            log::DEBUG("Cmd_Pause false timestamp: %f", raw.timestamp);
            last_motion_timestamp = last_observation_timestamp = raw.timestamp;
            mapper->resume();
            paused = false;
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_Lock) {
            log::DEBUG("Cmd_Lock %s timestamp: %f", static_cast<bool>(raw.sensor) ? "true" : "false", raw.timestamp);
            lock_map_data_t lock_data;
            lock_data.lock = raw.sensor;
            lock_data.reason
                = static_cast<sint_t>(raw.command.lock_reason) > static_cast<sint_t>(LockMapReason::Default)
                  ? LockMapReason::Default
                  : static_cast<LockMapReason>(raw.command.lock_reason);
            mapper->lock(lock_data);
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_Reset) {
            mapper->reset();
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_SetPose) {
            pose_t pose;
            pose.point.x.metre = raw.command.pose.x;
            pose.point.y.metre = raw.command.pose.y;
            pose.theta         = raw.command.pose.theta;
            mapper->set_pose(raw.timestamp, pose);
            valid_reset = last_observation_timestamp < 0;
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_SaveMap) {
            log::DEBUG("Cmd_Save %d timestamp: %f", raw.command.map_id, raw.timestamp);
            mapper->save_map(raw.command.map_id);
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_Load) {
            log::DEBUG("Cmd_Load %d timestamp: %f", raw.command.map_id, raw.timestamp);
            mapper->load_map(raw.command.map_id);
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::Cmd_RotateMap) {
            log::DEBUG("Cmd_RotateMap timestamp: %lf, (%f, %f, %f)", raw.timestamp, raw.command.pose.x, raw.command.pose.y, raw.command.pose.theta);
            auto pose          = pose_t{};
            pose.point.x.metre = raw.command.pose.x;
            pose.point.y.metre = raw.command.pose.y;
            pose.theta         = raw.command.pose.theta;
            mapper->rotate_map(pose);
            mapper->get_share_voxel_map(true);
            continue;
        }

        if (paused) {
            continue;
        }

        if (raw.type == rock::log_parser::LogDataType::SlamPose && raw.sensor == 0) {
            //            log::DEBUG("Recive slam pose: %lf", raw.timestamp);
            pose_t pose;
            pose.theta         = raw.position_2d.pos.theta;
            pose.point.x.metre = raw.position_2d.pos.x;
            pose.point.y.metre = raw.position_2d.pos.y;
            mapper->update_pose(pose, raw.timestamp);
        }

//        if (raw.type == rock::log_parser::LogDataType::Tof) {
//            auto observation = ObservationConverter::convert_to_tof(raw);
//            log::DEBUG("Recive tof: %lf, sensor: %d, type: %d", observation->timestamp, observation->cloud.sensor,
//                       observation->cloud.type);
//
//            mapper->update_observation(*observation);
//            ++count;
//
//            last_observation_timestamp = raw.timestamp;
//        }

        if (raw.type == rock::log_parser::LogDataType::Tof3DMap) {
//            if (use_tof[raw.sensor]++ % 4) {
//                continue;
//            }

            log::DEBUG("Recive tof: %lf, sensor: %d, type: %d", raw.timestamp, raw.sensor,
                     static_cast<int32_t>(raw.tof.exposure));
            auto observation = ObservationConverter::convert_to_tof(raw);
            log::DEBUG("convert tof: %lf, sensor: %d, type: %d", observation->timestamp, observation->sensor,
                     observation->type);
            mapper->update_point_cloud(*observation, RockPerceptorMessageType::Tof);
            ++count;
            last_observation_timestamp = raw.timestamp;
        }

        if (!mapper->run()){
            continue;
        }

        mapper->get_share_voxel_map();
    }

    mapper->save_map(1);
    mapper->get_share_voxel_map(true);
}