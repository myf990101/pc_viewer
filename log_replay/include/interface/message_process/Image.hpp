//
// Created by ray on 2024/2/8.
//

#pragma once
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include "Base.hpp"

namespace rock::log_replay::ros_wrapper {
    class Image : public SensorBase {
    public:
        void init(ros::NodeHandle& nh, const std::string& name, const std::string& frame = "") override {
            msg_header_.seq = 0;
            data_pub_       = nh.advertise<sensor_msgs::Image>(name, 100);
        }

        void process(sensor_data_t& data) override {
            ++msg_header_.seq;
            msg_header_.stamp.fromSec(data.timestamp);

            cv::Mat image;
            image = cv::imread(data.camera.name, cv::IMREAD_COLOR);

            sensor_msgs::ImagePtr msg;
            msg = cv_bridge::CvImage(msg_header_, "bgr8", image).toImageMsg();

            data_pub_.publish(*msg);
        }

        void process(cv::Mat& data, double time) {
            ++msg_header_.seq;
            msg_header_.stamp.fromSec(time);

            sensor_msgs::ImagePtr msg;
            msg = cv_bridge::CvImage(msg_header_, "bgr8", data).toImageMsg();

            data_pub_.publish(*msg);
        }

    private:
        std_msgs::Header msg_header_;
    };
}    // namespace rock::log_replay::ros_wrapper