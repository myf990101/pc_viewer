//
// Created by ray on 2024/2/8.
//

#pragma once
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "Base.hpp"
#include "utils/Utils.h"

namespace rock::log_replay::ros_wrapper {
    constexpr auto MapSize = float{40};

    class Map : public SensorBase {
    public:
        Map() = default;

        void init(ros::NodeHandle& nh, const std::string& name, const std::string& frame = "") override {
            is_init_          = false;
            last_update_time_ = 0;

            data_pub_       = nh.advertise<nav_msgs::OccupancyGrid>("/map", 100);
            map_update_pub_ = nh.advertise<map_msgs::OccupancyGridUpdate>("/map_updates", 100);

            map_.header.seq             = 0;
            map_.header.frame_id        = "map";
            map_.info.resolution        = 0.05;
            map_grid_                   = MapSize / map_.info.resolution;
            map_.info.width             = map_grid_;
            map_.info.height            = map_grid_;
            map_.info.origin.position.x = -MapSize / 2.0;
            map_.info.origin.position.y = -MapSize / 2.0;

            map_.data = std::vector<signed char>(map_.info.height * map_.info.width, 0);

            map_update_.header.seq      = 0;
            map_update_.header.frame_id = "map";

            reset();

            // data source
            pointcloud_sub_ = nh.subscribe(name, 10, &Map::pointcloud_callback, this);

            // r,t
            R_.w() = 1;
        }

        void process(sensor_data_t& data) override {
            T_.x() = data.slam_pose.current.x;
            T_.y() = data.slam_pose.current.y;
            R_     = rock::log_parser::euler_to_quaternion(data.slam_pose.current.theta);
        };

        void pointcloud_callback(const sensor_msgs::PointCloud2& data) {
            sensor_msgs::PointCloud cloud;
            sensor_msgs::convertPointCloud2ToPointCloud(data, cloud);
            for (const auto& p : cloud.points) {
                Eigen::Vector3f point = R_ * Eigen::Vector3f(p.x, p.y, p.z) + T_;

                int32_t x = (point.x() - map_.info.origin.position.x) / map_.info.resolution;
                int32_t y = (point.y() - map_.info.origin.position.y) / map_.info.resolution;

                if (x < 0 || y < 0 || x > map_grid_ || y > map_grid_) {
                    continue;
                }

                minx_ = std::min(minx_, x);
                miny_ = std::min(miny_, y);
                maxx_ = std::max(maxx_, x);
                maxy_ = std::max(maxy_, y);

                map_.data.at(y * map_grid_ + x) = 10;
            }

            publish(data.header.stamp.toSec());
        }

    private:
        void publish(double time) {
            if (!is_init_ && data_pub_.getNumSubscribers() > 0) {
                is_init_ = true;
                map_.header.stamp.fromSec(time);
                last_update_time_ = time;

                data_pub_.publish(map_);
                return;
            }

            if (time - last_update_time_ < 0.5) {
                return;
            }

            last_update_time_  = time;
            map_update_.width  = maxx_ - minx_ + 1;
            map_update_.height = maxy_ - miny_ + 1;
            map_update_.x      = minx_;
            map_update_.y      = miny_;

            std::vector<signed char> data;

            for (auto i = miny_; i <= maxy_; ++i) {
                for (auto j = minx_; j <= maxx_; ++j) { data.emplace_back(map_.data.at(i * map_.info.width + j)); }
            }

            if (data.empty()) {
                reset();
                return;
            }

            ++map_update_.header.seq;
            map_update_.header.stamp.fromSec(time);
            map_update_.data = data;

            map_update_pub_.publish(map_update_);

            reset();
        }

        void reset() {
            minx_ = miny_ = map_grid_;
            maxx_ = maxy_ = -map_grid_;
        }

    private:
        Eigen::Vector3f    T_;
        Eigen::Quaternionf R_;

        int32_t minx_;
        int32_t miny_;
        int32_t maxx_;
        int32_t maxy_;
        int32_t map_grid_;

        bool     is_init_;
        uint32_t last_update_time_;

        nav_msgs::OccupancyGrid       map_;
        map_msgs::OccupancyGridUpdate map_update_;

        ros::Subscriber pointcloud_sub_;
        ros::Publisher  map_update_pub_;
    };
}    // namespace rock::log_replay::ros_wrapper