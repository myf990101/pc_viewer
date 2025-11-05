//
// Created by gaoyuan on 2024/7/22.
//

#include "Mapping3dSim.h"
#include "interface/observation_converter.hpp"

namespace rock::log_replay {
    Mapping3dSim::Mapping3dSim() :
            RosInterface(),
            mapper_(new perceptor::mapping3d::TofMapper),
            map_counter_{},
            map_points_cache_{new point_cloud_t},
            map_points_cache_buffer_{new point_cloud_t},
            map_points_indexes_{},
            shared_points_cache_{new point_cloud_t},
            shared_points_indexes_{} {}

    void Mapping3dSim::init() {
        pose_wrapper_.init(nh(), "/pose");
        spot_wrapper_.init(nh(), "/tof_spot");
        flood_wrapper_.init(nh(), "/tof_flood");
        static_map_wrapper_.init(nh(), "/user_map");
        cloud_map_wrapper_.init(nh(), "user_map_3d", "map");
        marker_wrapper_.init(nh(), "used_rect_markers", "map");
        shared_map_wrapper_.init(nh(), "shared_map", "map");
        timestamp_wrapper_.init(nh(), "/time_display");

        std::vector<log_path_t> file_to_parse;
        get_file_list(file_to_parse);

        log_parser()->set_log_time(get_param("replay/log_start", 0));

        log_parser()->init(file_to_parse);

        log_parser()->begin();

        mapper_->init(get_param("map/tof_type", static_cast<sint_t>(TofHardware::OMS)),
                      get_param("replay/log_directory", std::string("")));
        mapper_->ivoxel_map()->option().need_fade = get_param("map/fade", true);
        auto debug_key = get_param("map/debug_key", std::vector<sint_t>(3, 0));
        mapper_->ivoxel_map()->option().debug_key = {debug_key.at(0), debug_key.at(1), debug_key.at(2)};
        mapper_->ivoxel_map()->option().fade_length = get_param("map/fade_length", 1);
        mapper_->ivoxel_map()->option().covered_count = get_param("map/horizon_count", 1);
    }

    void Mapping3dSim::run() {
        ROS_INFO_STREAM("Mapping3d sim node start");
        auto paused = false;
        auto rects = rects_3d_t{2};
        auto* buffer = new uint8_t[sizeof(perceptor::map::voxel::shared_map_t)];
        mapper_t::init_shared_memory(SharedMemoryType::SharedMap3d, buffer, sizeof(perceptor::map::voxel::shared_map_t));
        auto use_tof = std::array<uint64_t, 2>{};

        while (auto data = log_parser()->get_data()) {
            auto& raw = data->get();
            timestamp_wrapper_.process(raw);

            if (raw.type == log_data_type_t::Cmd_Pause && raw.sensor == 1) {
                ROS_INFO("Cmd_Pause true timestamp: %f", raw.timestamp);
                mapper_->pause();
                paused = true;
                continue;
            }

            if (raw.type == log_data_type_t::Cmd_Pause && raw.sensor == 0) {
                ROS_INFO("Cmd_Pause false timestamp: %f", raw.timestamp);
                mapper_->resume();
                paused = false;
                continue;
            }

            if (raw.type == log_data_type_t::Cmd_Lock) {
              ROS_INFO("Cmd_Lock %s timestamp: %f", static_cast<bool>(raw.sensor) ? "true" : "false", raw.timestamp);
                lock_map_data_t lock_data;
                lock_data.lock = raw.sensor;
                lock_data.reason
                    = static_cast<sint_t>(raw.command.lock_reason) > static_cast<sint_t>(LockMapReason::Default)
                          ? LockMapReason::Default
                          : static_cast<LockMapReason>(raw.command.lock_reason);
                mapper_->lock(lock_data);
                continue;
            }

            if (raw.type == log_data_type_t::Cmd_Reset) {
                mapper_->reset();
                map_points_cache_->clear();
                map_points_indexes_.clear();
                publish_map_points();
                publish_map_3d(buffer);
            }

            if (raw.type == log_data_type_t::Cmd_SetPose) {
                pose_t pose;
                pose.point.x.metre = raw.command.pose.x;
                pose.point.y.metre = raw.command.pose.y;
                pose.theta         = raw.command.pose.theta;
                mapper_->set_pose(raw.timestamp, pose);
            }

            if (raw.type == log_data_type_t::Cmd_SaveMap) {
                ROS_INFO("Cmd_Save %d timestamp: %f", raw.command.map_id, raw.timestamp);
                mapper_->save_map(raw.command.map_id);
            }

            if (raw.type == log_data_type_t::Cmd_Load) {
                ROS_INFO("Cmd_Load %d timestamp: %f", raw.command.map_id, raw.timestamp);
                mapper_->load_map(raw.command.map_id);
                publish_map_points();
                publish_map_3d(buffer, true);
            }

            if (raw.type == log_data_type_t::Map) {
                static_map_wrapper_.process(raw);
                continue;
            }

            if (raw.type == log_data_type_t::Cmd_RotateMap) {
                ROS_INFO("Cmd_RotateMap timestamp: %lf, (%f, %f, %f)", raw.timestamp, raw.command.pose.x, raw.command.pose.y, raw.command.pose.theta);
                auto pose          = pose_t{};
                pose.point.x.metre = raw.command.pose.x;
                pose.point.y.metre = raw.command.pose.y;
                pose.theta         = raw.command.pose.theta;
                mapper_->rotate_map(pose);
                map_points_cache_->clear();
                map_points_indexes_.clear();
                publish_map_points();
                mapper_->get_share_voxel_map(true);
                publish_map_3d(buffer, true);
            }

            if (paused) {
                continue;
            }

            if (raw.type == log_data_type_t::SlamPose) {
                map_wrapper_.process(raw);
                pose_wrapper_.process(raw);
                pose_t pose;
                pose.theta         = raw.position_2d.pos.theta;
                pose.point.x.metre = raw.position_2d.pos.x;
                pose.point.y.metre = raw.position_2d.pos.y;
                mapper_->update_pose(pose, raw.timestamp);
            }

            if (raw.type == log_data_type_t::Tof && get_param("default/bin14", false)) {
                use_tof[raw.sensor] = use_tof[raw.sensor]++ % 4;

                if (use_tof[raw.sensor]++ % 1) {
                    continue;
                }
                auto observation = ObservationConverter::convert_to_tof(raw);
                ROS_INFO("Recive tof: %lf, sensor: %d, type: %d", observation->timestamp, observation->sensor,
                         observation->type);
                mapper_->update_point_cloud(*observation, RockPerceptorMessageType::Tof);
            }

            if (raw.type == log_data_type_t::Tof3DMap) {
                if (use_tof[raw.sensor]++ % 1 || raw.tof.exposure == log_parser::Tof::Exposure::HdrSpot) {
                    continue;
                }
                ROS_INFO("Recive tof: %lf, sensor: %d, type: %d", raw.timestamp, raw.sensor,
                         static_cast<int32_t>(raw.tof.exposure));
                auto observation = ObservationConverter::convert_to_tof(raw);
                ROS_INFO("convert tof: %lf, sensor: %d, type: %d", observation->timestamp, observation->sensor,
                         observation->type);
                mapper_->update_point_cloud(*observation, RockPerceptorMessageType::Tof);
            }

            if (!mapper_->run()){
                continue;
            }

//            if (raw.tof.exposure == log_parser::Tof::Exposure::HdrSpot) {
//                spot_wrapper_.process_tof_raw(raw);
//            } else if (raw.tof.exposure == log_parser::Tof::Exposure::HdrFlood) {
//                flood_wrapper_.process_tof_raw(raw);
//            }

            if (static_cast<int32_t>(mapper_->measurement_group()->cloud_body->type()) == static_cast<int32_t>(tof_t::Exposure::HdrFlood)) {
                flood_wrapper_.process(mapper_->measurement_group()->cloud_body, true);
            } else if (static_cast<int32_t>(mapper_->measurement_group()->cloud_body->type()) == static_cast<int32_t>(tof_t::Exposure::HdrSpot)) {
                spot_wrapper_.process(mapper_->measurement_group()->cloud_body, true);
            }

            mapper_->get_share_voxel_map();
            rects.at(0) = mapper_->map_used();
            rects.at(1) = mapper_->map_updated();
            marker_wrapper_.process(rects, raw.timestamp);

            publish_map_points();
            publish_map_3d(buffer);
        }

        mapper_->save_map(1);
        mapper_->get_share_voxel_map(true);
    }

    template<uint32_t Scale_t>
    auto Mapping3dSim::vectorize(point_3d_grid_t key) const -> point_3d_metre_t {
        static const auto VectorizeScale = 1 / static_cast<metre_t>(Scale_t);
        return {static_cast<metre_t>(key.x()) * VectorizeScale, static_cast<metre_t>(key.y()) * VectorizeScale,
            static_cast<metre_t>(key.z()) * VectorizeScale};
    }

    void Mapping3dSim::publish_map_points() {
        static auto free_count = get_param("map/map_counter", 0);
        auto&       map        = mapper_->ivoxel_map();
        auto&       map_data   = map->data();
        auto const& map_update = map->map_points_update();
        ROS_INFO("map update size = %ld", map_update.size());

        for (auto const& key : map_update) {
            if (!map_data[key])
                continue;

            if (map_points_indexes_.find(key) == map_points_indexes_.end()) {
                map_points_indexes_[key] = map_points_cache_->size();
                map_points_cache_->push_back(point_t(vectorize<mapper_t::ivoxel_map_t::MapMetreToGrid>(key)));
            }

            auto  index       = map_points_indexes_[key];
            auto& point       = map_points_cache_->points()[index];
            point.intensity() = map_data[key]->cell.value();
        }

        map_points_cache_->resize(map_points_cache_->size());

        ++map_counter_;
        cloud_map_wrapper_.process(map_points_cache_);

        if (map_counter_ < free_count)
            return;
        map_counter_ = 0;

        auto removed = size_t{};
        map_points_cache_buffer_->clear();
        map_points_cache_buffer_->reserve(map_points_cache_->size());

        for (auto it = map_points_indexes_.begin(); it != map_points_indexes_.end();) {
            auto const& key          = it->first;
            assert(it->second < map_points_cache_->size());
            auto&       point_cached = map_points_cache_->at(it->second);

            if (map_data.valid(key)) {
                point_cached.intensity() = map_data[key]->cell.value();
                it->second = map_points_cache_buffer_->size();
                map_points_cache_buffer_->push_back(point_cached);
                ++it;
                continue;
            }

            ++removed;
            it = map_points_indexes_.erase(it);
        }

        map_points_cache_buffer_->resize(map_points_cache_buffer_->size());
        map_points_cache_.swap(map_points_cache_buffer_);
        cloud_map_wrapper_.process(map_points_cache_);

        ROS_INFO("stable size: After optimization, removed: %lu, map_points_indexes: %lu, map_points_cache: %i", removed,
                  map_points_indexes_.size(), map_points_cache_->size());
    }

    void Mapping3dSim::publish_map_3d(void* buffer, bool total) {
        auto* map      = static_cast<perceptor::map::voxel::shared_map_t*>(buffer);
        auto  rect     = total ? mapper_->map_used() : mapper_->map_updated();
        auto  offset   = perceptor::map::voxel::SharedMapLength / 2;
        auto& map_data = mapper_->ivoxel_map()->data();

        for (auto y = rect.front(); y <= rect.back(); ++y) {
            auto index_y = y + offset;

            for (auto x = rect.left(); x <= rect.right(); ++x) {
                auto index_x = x + offset;
                auto index = index_y * perceptor::map::voxel::SharedMapLength + index_x;
                auto cell = std::bitset<perceptor::map::voxel::MaxHeightKey>(map->cells[index]);

                for (auto z = std::max(0, rect.bottom());
                     z < std::min(perceptor::map::voxel::MaxHeightKey, rect.top() + 1); ++z) {
                    if (!cell[z]) {
                        continue;
                    }

                    auto key = map_key_t{x, y, z};

                    if (!map_data[key]) continue;

                    if (shared_points_indexes_.find(key) == shared_points_indexes_.end()) {
                        shared_points_indexes_[key] = shared_points_cache_->size();
                        shared_points_cache_->push_back(point_t(vectorize<mapper_t::ivoxel_map_t::MapMetreToGrid>(key)));
                    }

                    auto  key_index   = shared_points_indexes_[key];
                    auto& point       = shared_points_cache_->points()[key_index];
                    point.intensity() = map_data[key]->cell.value();
                }
            }
        }
        shared_points_cache_->resize(shared_points_cache_->size());
        shared_map_wrapper_.process(shared_points_cache_);
    }
}