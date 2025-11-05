//
// Created by ray on 2024/2/26.
//

#pragma once
#include <std_msgs/String.h>

#include "Base.hpp"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "common/Types.h"
#include "common/Rectangle3D.h"

namespace rock::log_replay::ros_wrapper {
    class Marker : public SensorBase {

        using rect_3d_t   = perceptor::common::Rectangle3D;
        using rects_3d_t  = std::vector<rect_3d_t>;

    public:
        void init(ros::NodeHandle& nh, const std::string& name, const std::string& frame = "") {
            if (name.find("markers") != std::string::npos) {
                data_pub_ = nh.advertise<visualization_msgs::MarkerArray>(name, 100);
            } else {
                data_pub_ = nh.advertise<std_msgs::String>(name, 100);
            }
        }

        void process(sensor_data_t& data) override {
            string_.data = std::to_string(data.timestamp);

            if (data_pub_.getNumSubscribers()) {
                data_pub_.publish(string_);
            }
        }

        void process(rects_3d_t const& rects, timestamp_t timestamp) {
            markers_.markers.clear();
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/map";
            marker.header.stamp.fromSec(timestamp);
            marker.ns = "rect";
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.color.r = 0.f;
            marker.color.g = 0.f;
            marker.color.b = 0.2f;
            marker.color.a = 0.2f;

            for (auto i = 0; i < rects.size(); ++i) {
                if (rects.at(i).empty()) {
                    continue;
                }

                marker.id = i;
                marker.pose.position.x = rects.at(i).center().x.grid * 0.05;
                marker.pose.position.y = rects.at(i).center().y.grid * 0.05;
                marker.pose.position.z = rects.at(i).center().z.grid * 0.05;
                marker.pose.orientation.x = 0;
                marker.pose.orientation.y = 0;
                marker.pose.orientation.z = 0;
                marker.pose.orientation.w = 1;

                marker.scale.x = rects.at(i).length() * 0.05;
                marker.scale.y = rects.at(i).width() * 0.05;
                marker.scale.z = rects.at(i).height() * 0.05;

                marker.color.b *= static_cast<float>(1 + 2 * i);
                marker.color.a *= static_cast<float>(1 + i);
                marker.lifetime = ros::Duration();
                markers_.markers.emplace_back(marker);
            }

            data_pub_.publish(markers_);
        }

    private:
        std_msgs::String string_;

        visualization_msgs::MarkerArray markers_;
    };
}    // namespace rock::log_replay::ros_wrapper