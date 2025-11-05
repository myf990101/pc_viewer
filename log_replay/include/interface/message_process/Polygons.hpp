//
// Created by ray on 2024/2/8.
//

#pragma once

#include <jsk_recognition_msgs/PolygonArray.h>

#include "Base.hpp"

namespace rock::log_replay::ros_wrapper {
    class Polygons : public SensorBase {
    public:
        Polygons() = default;

        void init(ros::NodeHandle& nh, const std::string& name, const std::string& frame = "") override {
            polys_.header.seq = 0;
            if (frame == "") {
                frame_ = "/map";

                polys_.header.frame_id = "/map";
            } else {
                frame_ = frame;

                polys_.header.frame_id = frame;
            }
            data_pub_ = nh.advertise<jsk_recognition_msgs::PolygonArray>(name, 100);
        }

        void process(sensor_data_t& data) override {
            auto polygon_id = 0;

            geometry_msgs::PolygonStamped poly;
            poly.header.frame_id = frame_;
            poly.header.stamp.fromSec(data.timestamp);

            ++polys_.header.seq;
            polys_.header.stamp.fromSec(data.timestamp);
            polys_.polygons.clear();

            for (const auto& p : data.nav.points) {
                if (p.z != polygon_id) {
                    ++polygon_id;

                    polys_.polygons.emplace_back(poly);
                    poly.polygon.points.clear();
                }

                geometry_msgs::Point32 point;
                point.x = p.x;
                point.y = p.y;
                poly.polygon.points.emplace_back(point);
            }

            if (data_pub_.getNumSubscribers()) {
                data_pub_.publish(polys_);
            }
        }

    private:
        std::string                        frame_;
        jsk_recognition_msgs::PolygonArray polys_;
    };
}    // namespace rock::log_replay::ros_wrapper