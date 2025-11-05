#include "InvasionSim.h"

#include "util/Timer.h"

namespace rock::log_replay {
    InvasionSim::InvasionSim() : RosInterface() {}

    void InvasionSim::init() {
        invasion_detector_ = std::make_unique<perceptor::detection::invasion::InvasionDetector>();
        top_tof_classify_  = std::make_unique<perceptor::detection::topclassify::TopClassify>();

        cone_wrapper_.init(nh(), "/cone", "base_link");
        cone_wrapper2_.init(nh(), "/cone2", "base_link");
        cone_wrapper3_.init(nh(), "/cone3", "base_link");
        cone_wrapper4_.init(nh(), "/cone4", "base_link");
        pose_wrapper_.init(nh(), "/pose");
        origin_wrapper_.init(nh(), "/origin_image");
        flood_wrapper_.init(nh(), "/flood");
        classify_wrapper_.init(nh(), "/classify");
        marker_wrapper_.init(nh(), "/time_display");
        invasion_wrapper_.init(nh(), "/invasion");

        std::vector<log_path_t> file_to_parse;
        get_file_list(file_to_parse);

        log_parser()->set_log_time(get_param("replay/log_start", 0));

        log_parser()->init(file_to_parse);

        log_parser()->begin();
    }

    void InvasionSim::process_cones(uint32_t timestamp, points_t cube,
                                    rock::log_replay::ros_wrapper::Polygon& cone_wrapper) {
        if (cube.empty()) {
            vec3f_t pt1;
            cube.push_back(pt1);
            vec3f_t pt2;
            cube.push_back(pt2);
            cube[0].x() = 0;
            cube[0].y() = 0;
            cube[0].z() = 0;
            cube[1].x() = 0;
            cube[1].y() = 0;
            cube[1].z() = 0;
        }
        sensor_data_t       polygon;
        log_parser::Point3D pt;
        polygon.timestamp = timestamp;

        pt.x = cube[0].x();
        pt.y = cube[0].y();
        pt.z = cube[0].z();
        polygon.nav.points.emplace_back(pt);

        pt.x = cube[1].x();
        pt.y = cube[0].y();
        pt.z = cube[0].z();
        polygon.nav.points.emplace_back(pt);

        pt.x = cube[1].x();
        pt.y = cube[1].y();
        pt.z = cube[0].z();
        polygon.nav.points.emplace_back(pt);

        pt.x = cube[0].x();
        pt.y = cube[1].y();
        pt.z = cube[0].z();
        polygon.nav.points.emplace_back(pt);

        polygon.nav.points.emplace_back(polygon.nav.points.front());

        pt.x = cube[0].x();
        pt.y = cube[0].y();
        pt.z = cube[1].z();
        polygon.nav.points.emplace_back(pt);

        pt.x = cube[1].x();
        pt.y = cube[0].y();
        pt.z = cube[1].z();
        polygon.nav.points.emplace_back(pt);

        pt.x = cube[1].x();
        pt.y = cube[1].y();
        pt.z = cube[1].z();
        polygon.nav.points.emplace_back(pt);

        pt.x = cube[0].x();
        pt.y = cube[1].y();
        pt.z = cube[1].z();
        polygon.nav.points.emplace_back(pt);

        polygon.nav.points.emplace_back(polygon.nav.points[5]);
        polygon.nav.points.emplace_back(polygon.nav.points[6]);
        polygon.nav.points.emplace_back(polygon.nav.points[1]);
        polygon.nav.points.emplace_back(polygon.nav.points[2]);
        polygon.nav.points.emplace_back(polygon.nav.points[7]);
        polygon.nav.points.emplace_back(polygon.nav.points[8]);
        polygon.nav.points.emplace_back(polygon.nav.points[3]);

        cone_wrapper.process(polygon);
    }

    void InvasionSim::run() {
        ROS_INFO("begin ==============");

        using pointcloud_4d
            = rock::perceptor::common::PointCloud<rock::perceptor::common::PointXYZIType<metre_t>>;
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

                invasion_detector_->update_pose(pose);

                continue;
            }

            if (raw.type == log_data_type_t::AvoidCube) {
                top_tof_classify_->update_cube(raw.avoid_cube.armworkstatus, raw.avoid_cube.armPosition,
                                               raw.avoid_cube.cube, raw.avoid_cube.seed);
                invasion_detector_->update_arm_working_status(raw.avoid_cube.armworkstatus);

                process_cones(raw.timestamp, top_tof_classify_->get_cube(), cone_wrapper_);
                process_cones(raw.timestamp, top_tof_classify_->get_grasp_cube(), cone_wrapper2_);
                continue;
            }

            if (raw.type == log_data_type_t::Tof && raw.tof.exposure == tof_t::Exposure::Flood
                && raw.sensor == static_cast<int>(tof_t::Sensor::Top)) {
                flood_wrapper_.process(raw);
                invasion_detector_->update_arm_state(true);

                auto observation = ObservationConverter::convert_to_tof(raw);

                observation->reserved[1] = observation->size;
                std::cout << "observation->reserved[1]:" << observation->reserved[1] << std::endl;

                top_tof_classify_->update_pointcloud(*observation);
                top_tof_classify_->classify();

                pointcloud_4d::ptr_t classified_cloud = top_tof_classify_->get_valid_cloud();

                for (auto i = 0; i < classified_cloud->size(); ++i) {
                    (*classified_cloud).points()[i].type() = (*classified_cloud).points()[i].id() + 1;
                }

                classify_wrapper_.process(classified_cloud);
                auto tof_data = top_tof_classify_->get_pre_invasion_points();
                pre_invasion_info_t* tof_data_invasion = ((pre_invasion_info_t*)rock_perceptor_memory_allocate(sizeof(pre_invasion_info_t) + sizeof(point_classify_t) * tof_data->size));
                tof_data_invasion->timestamp = tof_data->timestamp;
                tof_data_invasion->pre_invasion_num = tof_data->reserved[0];
                tof_data_invasion->arm_and_pre_invasion_num = tof_data->reserved[1];
                std::memcpy(&tof_data_invasion->cloud, &tof_data->points_classify, sizeof(point_cloud_data_t) + sizeof(point_classify_t) * tof_data->size);
                invasion_detector_->update_pre_invasion_info(tof_data_invasion);
                

                invasion_detector_->invasion_detection();

                auto invasion_cloud = invasion_detector_->get_invasion_cloud();

                if (invasion_cloud->size() > 0) {
                    invasion_wrapper_.process(invasion_cloud);
                }

                auto invasion_state = invasion_detector_->get_invasion_state();
                auto move_state     = invasion_detector_->get_move_state();
                ROS_DEBUG("invasion_state:%d move_state:%d", (int)invasion_state, (int)move_state);
                rock::perceptor::util::Timer::PrintAll();
                continue;
            }

            if (raw.type == log_data_type_t::Camera_Image) {
                origin_wrapper_.process(raw);
                continue;
            }

            if (raw.type == log_data_type_t::OdoGyro) {
                motion_t motion;
                motion.timestamp          = raw.timestamp;
                motion.pose.point.x.metre = raw.slam_pose.current.x;
                motion.pose.point.y.metre = raw.slam_pose.current.y;
                motion.pose.theta         = raw.slam_pose.current.theta;
                motion.gyro.euler.pitch   = raw.odo_gyro.euler.pitch;
                motion.gyro.euler.roll    = raw.odo_gyro.euler.roll;
                motion.gyro.euler.yaw     = raw.odo_gyro.euler.yaw;
                motion.velocity.v         = raw.odo_gyro.actual_velocity.v;
                motion.velocity.w         = raw.odo_gyro.actual_velocity.w;
                invasion_detector_->update_odom(motion);
                ROS_DEBUG("insert odom_v %.3f ", motion.velocity.v);
                ROS_DEBUG("insert odom_w %.3f ", motion.velocity.w);
                ROS_DEBUG("insert odom %.3f ", motion.timestamp);
                continue;
            }
        }

        ROS_INFO("end ==============");
    }
}    // namespace rock::log_replay
