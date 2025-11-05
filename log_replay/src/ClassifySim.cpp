#include "ClassifySim.h"

namespace rock::log_replay {
    ClassifySim::ClassifySim() : RosInterface() {}

    auto ClassifySim::compute_delta_rotation(const Eigen::Vector3d& n1, const Eigen::Vector3d& n2) -> Eigen::Matrix3d {
        Eigen::Vector3d axis = n1.cross(n2);
        axis /= axis.norm();
        return Eigen::AngleAxisd(std::acos(n1.dot(n2) / (n1.norm() * n2.norm())), axis).toRotationMatrix();
    }

    auto ClassifySim::process_tof_slc(log_parser::Nav::TofSLC& data) -> sensor_extrinsic_t {
        Eigen::Vector3d t_bt{data.trans[0], data.trans[1], data.gnd[3]};

        Eigen::Matrix3d Rbt(Eigen::AngleAxisd(-M_PI / 2 + data.yaw, Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()));

        Eigen::Vector3d normal_t{data.gnd[0], data.gnd[1], data.gnd[2]};
        if (data.version == 0) {
            normal_t[0] = data.gnd_rect[0];
            normal_t[1] = data.gnd_rect[1];
            normal_t[2] = data.gnd_rect[2];
        }

        Eigen::Vector3d normal_b = Rbt * normal_t;

        Eigen::Vector3d axis_z{0, 0, 1};

        auto delta_R = compute_delta_rotation(normal_b, axis_z);    // Tbb'

        Eigen::Matrix3d Rbt_calib = delta_R * Rbt;    // Tbt' = Tbb' * Tb't'

        return {
            .sensor = static_cast<perceptor::common::CalibrationType>(data.sensor), .trans = t_bt, .rot = Rbt_calib};
    }

    void ClassifySim::prepare_reference_points() {
        for (auto i = 0; i <= 6; ++i) {
            auto distance = 0.1 * i;
            for (auto j = -50; j <= 50; j += 2) {
                reference_points_.emplace_back(TofPosition + distance * std::cos(j / 57.3),
                                               distance * std::sin(j / 57.3), 0);
            }
        }
    }

    void ClassifySim::init() {
        algorithm_ = std::make_unique<tof_classify_type>();

        if (get_param("replay/sensor_type", 0)) {
            algorithm_->set_sensor_type(TofHardware::ChaoFeng);
        } else {
            algorithm_->set_sensor_type(TofHardware::OMS);
        }

        pose_wrapper_.init(nh(), "/pose");
        image_wrapper_.init(nh(), "/image");
        flood_wrapper_.init(nh(), "/flood");
        marker_wrapper_.init(nh(), "/time_display");
        classify_wrapper_.init(nh(), "/classify");
        reference_wrapper_.init(nh(), "/reference");
        prepare_reference_points();

        std::vector<log_path_t> file_to_parse;
        get_file_list(file_to_parse);

        log_parser()->set_log_time(get_param("replay/log_start", 0));

        log_parser()->init(file_to_parse);

        log_parser()->begin();
    }

    void ClassifySim::run() {
        ROS_INFO("begin ==============");

        auto last_observation_timestamp = double{-1};

        while (auto data = log_parser()->get_data()) {
            auto& raw = data->get();
            marker_wrapper_.process(raw);

            if (raw.type == log_data_type_t ::SlamPose) {
                pose_wrapper_.process(raw);
                pose_2d_stamped_t pose;
                pose.timestamp          = raw.timestamp;
                pose.pose.theta         = raw.position_2d.pos.theta;
                pose.pose.point.x.metre = raw.position_2d.pos.x;
                pose.pose.point.y.metre = raw.position_2d.pos.y;

                algorithm_->update_pose(pose);

                continue;
            }

            if (raw.type == log_data_type_t::Nav && raw.sensor == static_cast<int>(nav_t::Type::TofSLC)) {
                sensor_extrinsic_t param[3];
                param[0] = process_tof_slc(raw.nav.TofSLCs[0]);
                param[1] = process_tof_slc(raw.nav.TofSLCs[1]);
                algorithm_->set_tof_param(param);
            }

            if (raw.type == log_data_type_t::Tof && raw.tof.exposure == tof_t::Exposure::HdrFlood) {
                if (last_observation_timestamp > 0 && raw.timestamp - last_observation_timestamp > 10) {
                    ROS_DEBUG("Lost ToF msg for too long time, "
                              "last timestamp: %f, current timestamp: %f, diff timestamp: %f, finish system!",
                              last_observation_timestamp, raw.timestamp, raw.timestamp - last_observation_timestamp);
                    break;
                }

                flood_wrapper_.process(raw);

                reference_wrapper_.process(reference_points_, raw.timestamp);

                auto observation = ObservationConverter::convert_to_tof(raw);
                algorithm_->update_point_cloud(*observation);
                last_observation_timestamp = raw.timestamp;

                if (!algorithm_->run()) {
                    continue;
                }

                classify_wrapper_.process(algorithm_->get_cloud());
            }

            if (raw.type == log_data_type_t::Camera_Image) {
                image_wrapper_.process(raw);
            }

        }

        ROS_INFO("end ==============");
    }
}    // namespace rock::log_replay
