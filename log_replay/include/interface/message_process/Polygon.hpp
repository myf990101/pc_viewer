//
// Created by ray on 2024/2/8.
//

#pragma once

#include <jsk_recognition_msgs/PolygonArray.h>

#include "Base.hpp"

namespace rock::log_replay::ros_wrapper {
    class Polygon : public SensorBase {
    public:
        Polygon() = default;

        void init(ros::NodeHandle& nh, const std::string& name, const std::string& frame = "") override {
            poly_.header.seq = 0;
            if (frame == "") {
                poly_.header.frame_id = "/map";
            } else {
                poly_.header.frame_id = frame;
            }
            data_pub_ = nh.advertise<geometry_msgs::PolygonStamped>(name, 100);
        }

        void process(sensor_data_t& data) override {
            ++poly_.header.seq;
            poly_.polygon.points.clear();
            poly_.header.stamp.fromSec(data.timestamp);

            for (const auto& p : data.nav.points) {
                geometry_msgs::Point32 point;
                point.x = p.x;
                point.y = p.y;
                point.z = p.z;
                poly_.polygon.points.emplace_back(point);
            }

            if (data_pub_.getNumSubscribers()) {
                data_pub_.publish(poly_);
            }
        }

    private:
        geometry_msgs::PolygonStamped poly_;
    };
}    // namespace rock::log_replay::ros_wrapper