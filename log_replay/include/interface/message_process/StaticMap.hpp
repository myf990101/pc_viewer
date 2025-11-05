//
// Created by ray on 2024/2/8.
//

#pragma once
#include <geometry_msgs/TransformStamped.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

#include "Base.hpp"

namespace rock::log_replay::ros_wrapper {
    constexpr auto Scale = int{20};

    class StaticMap : public SensorBase {
    public:
        struct ViewHeader {
            static const char* const Signature;

            char     signature[6];
            uint16_t version;
            uint16_t size;
        } __attribute__((packed));

        struct ViewInfoV2 {
            int scale;
            int left;
            int top;
            int width;
            int height;
        } __attribute__((packed));

    public:
        void init(ros::NodeHandle& nh, const std::string& name, const std::string& frame = "") override {
            data_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/static_map", 100);

            map_.header.seq      = 0;
            map_.header.frame_id = "map";
            map_.info.resolution = 1.0 / Scale;
        }

        void process(sensor_data_t& data) override {
            std::ifstream fs(data.map.location);

            ViewInfoV2               map_info;
            std::vector<signed char> map_data;

            if (load_map(fs, map_data, map_info)) {
                map_.header.stamp = ros::Time::now();

                map_.info.width                = map_info.width;
                map_.info.height               = map_info.height;
                map_.info.origin.position.x    = map_info.left * map_.info.resolution - 0.5 * map_.info.resolution;
                map_.info.origin.position.y    = map_info.top * map_.info.resolution - 0.5 * map_.info.resolution;
                map_.info.origin.orientation.w = 1.0;

                map_.data.resize(map_data.size());
                for (auto y = 0; y < map_info.height; ++y) {
                    auto offset = y * map_info.width;
                    auto start  = map_data.begin() + offset;
                    std::transform(start, start + map_info.width, map_.data.data() + offset, [&](const uint8_t& src) {
                        return src <= 120 ? 0 : src >= 136 ? 100 : -1;
                    });
                }

                while (!data_pub_.getNumSubscribers()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }

                data_pub_.publish(map_);
            }
        }

    private:
        auto inline load_map(std::istream& is, std::vector<signed char>& view, ViewInfoV2& info) -> bool {
            if (is.bad()) {
                return false;
            }

            ViewHeader header{};
            is.read((std::istream::char_type*)&header, sizeof(ViewHeader));
            if (is.bad()) {
                return false;
            }

            if (header.version == 2 && header.size == sizeof(ViewInfoV2)) {
                is.read((std::istream::char_type*)&info, sizeof(ViewInfoV2));
                if (is.bad()) {
                    return false;
                }

                if (info.scale != Scale) {
                    return false;
                }

                view.resize(info.width * info.height);
                is.read((std::istream::char_type*)view.data(), sizeof(uint8_t) * view.size());
                return true;
            }

            return false;
        }

    private:
        nav_msgs::OccupancyGrid map_;
    };
}    // namespace rock::log_replay::ros_wrapper