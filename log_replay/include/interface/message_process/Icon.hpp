//
// Created by ray on 2024/2/8.
//

#pragma once
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>

#include <bitset>

#include "Base.hpp"

namespace rock::log_replay::ros_wrapper {
    constexpr auto MaxIcon = int(500);

    class Icon : public SensorBase {
    public:
        Icon() : icons_(MaxIcon) {}

        void init(ros::NodeHandle& nh, const std::string& name, const std::string& frame = "") override {
            valid_count_                 = 0;
            icon_points_.header.seq      = 0;
            icon_points_.header.frame_id = "map";

            data_pub_ = nh.advertise<sensor_msgs::PointCloud>(name, 100);
        }

        void process(sensor_data_t& data) override {
            if (data.icon.remove_id < 0) {
                ++valid_count_;
                for (auto i = 0; i < MaxIcon; ++i) {
                    if (!flag_[i]) {
                        flag_[i] = true;

                        icons_[i].x = data.icon.x / 1000.;
                        icons_[i].y = data.icon.y / 1000.;
                        break;
                    }
                }
            } else {
                flag_[data.icon.remove_id] = false;
            }

            icon_points_.points.clear();
            for (auto i = 0; i < valid_count_; ++i) {
                if (flag_[i]) {
                    icon_points_.points.emplace_back(icons_[i]);
                }
            }

            icon_points_.header.stamp.fromSec(data.timestamp);
            ++icon_points_.header.seq;

            if (data_pub_.getNumSubscribers()) {
                data_pub_.publish(icon_points_);
            }
        }

    private:
        int                                 valid_count_;
        std::bitset<MaxIcon>                flag_;
        std::vector<geometry_msgs::Point32> icons_;

        sensor_msgs::PointCloud icon_points_;
    };
}    // namespace rock::log_replay::ros_wrapper