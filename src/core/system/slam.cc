//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.h"
#include "core/g2p5/g2p5.h"
#include "core/lightning_math.hpp"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <utility>

namespace lightning {

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

bool SlamSystem::Init(const std::string& yaml_path) {
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio module";
        return false;
    }

    auto yaml = YAML::LoadFile(yaml_path);
    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    options_.pub_tf_ = yaml["system"]["pub_tf"] ? yaml["system"]["pub_tf"].as<bool>() : false;
    options_.pub_odom_ = yaml["system"]["pub_odom"] ? yaml["system"]["pub_odom"].as<bool>() : false;
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();
    options_.log_pose_opt_ = yaml["system"]["log_pose_opt"] ? yaml["system"]["log_pose_opt"].as<bool>() : false;
    options_.enable_lidar_rviz_ = yaml["system"]["enable_lidar_loc_rviz"] ? yaml["system"]["enable_lidar_loc_rviz"].as<bool>() : false;
    options_.enable_path_rviz_ = yaml["system"]["enable_path_rviz"] ? yaml["system"]["enable_path_rviz"].as<bool>() : false;
    options_.use_imu_init_ = yaml["system"]["use_imu_orient"] ? yaml["system"]["use_imu_orient"].as<bool>() : false;
    options_.async_lio_ = yaml["system"]["async_lio"] ? yaml["system"]["async_lio"].as<bool>() : true;
    const int max_pending_lidar = yaml["system"]["async_lio_max_pending_lidar"]
                                      ? yaml["system"]["async_lio_max_pending_lidar"].as<int>()
                                      : 192;
    const int max_pending_imu = yaml["system"]["async_lio_max_pending_imu"]
                                    ? yaml["system"]["async_lio_max_pending_imu"].as<int>()
                                    : 20000;
    options_.async_lio_max_pending_lidar_ = static_cast<std::size_t>(std::max(1, max_pending_lidar));
    options_.async_lio_max_pending_imu_ = static_cast<std::size_t>(std::max(1, max_pending_imu));
    if(options_.enable_lidar_rviz_ && !options_.enable_path_rviz_) {
        options_.enable_path_rviz_ = true; // 发布路径时会包含位姿信息，方便调试
    }

    LOG(INFO) << "SlamSystem Options: "
              << "\n  online_mode: " << options_.online_mode_
              << "\n  with_cc: " << options_.with_cc_
              << "\n  with_gridmap: " << options_.with_gridmap_
              << "\n  with_loop_closing: " << options_.with_loop_closing_
              << "\n  with_visualization: " << options_.with_visualization_
              << "\n  with_2dvisualization: " << options_.with_2dvisualization_
              << "\n  pub_odom: " << options_.pub_odom_
              << "\n  pub_tf: " << options_.pub_tf_
              << "\n  enable_lidar_rviz: " << options_.enable_lidar_rviz_
              << "\n  enable_path_rviz: " << options_.enable_path_rviz_
              << "\n  async_lio: " << options_.async_lio_
              << "\n  async_lio_max_pending_lidar: " << options_.async_lio_max_pending_lidar_
              << "\n  async_lio_max_pending_imu: " << options_.async_lio_max_pending_imu_
              << "\n  step_on_kf: " << options_.step_on_kf_
              << "\n  log_pose_opt: " << options_.log_pose_opt_
              << "\n  use_imu_init: " << options_.use_imu_init_;

    if (yaml["system"]["map_path"]) {
        std::string map_path = yaml["system"]["map_path"].as<std::string>();
        if (map_path.back() == '/') {
            map_path.pop_back();
        }
        size_t last_slash = map_path.find_last_of('/');
        if (last_slash != std::string::npos) {
            map_name_ = map_path.substr(last_slash + 1);
        } else {
            map_name_ = map_path;
        }
    } else {
        map_name_ = "new_map";
    }
    LOG(INFO) << "map name: " << map_name_;

    /// loop closing
    if (options_.with_loop_closing_) {
        LOG(INFO) << "slam with loop closing";
        LoopClosing::Options options;
        options.online_mode_ = options_.online_mode_;
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->Init(yaml_path);
    }

    if (options_.with_visualization_) {
        LOG(INFO) << "slam with 3D UI";
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();

        lio_->SetUI(ui_);
    }

    if (options_.with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);

        if (options_.with_loop_closing_) {
            /// 当发生回环时，触发一次重绘
            lc_->SetLoopClosedCB([this]() { g2p5_->RedrawGlobalMap(); });
        }

        if (options_.with_2dvisualization_) {
            g2p5_->SetMapUpdateCallback([this](g2p5::G2P5MapPtr map) {
                cv::Mat image = map->ToCV();
                cv::imshow("map", image);

                if (options_.step_on_kf_) {
                    cv::waitKey(0);

                } else {
                    cv::waitKey(10);
                }
            });
        }
    }

    if (options_.online_mode_) {
        LOG(INFO) << "online mode, creating ros2 node ... ";

        /// subscribers
        node_ = std::make_shared<rclcpp::Node>("lightning_slam");

        imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
        cloud_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
        livox_topic_ = yaml["common"]["livox_lidar_topic"].as<std::string>();

        const int imu_qos_depth =
            std::max(1, yaml["system"]["imu_qos_depth"] ? yaml["system"]["imu_qos_depth"].as<int>() : 2000);
        const int lidar_qos_depth =
            std::max(1, yaml["system"]["lidar_qos_depth"] ? yaml["system"]["lidar_qos_depth"].as<int>() : 20);
        const bool imu_qos_reliable =
            yaml["system"]["imu_qos_reliable"] ? yaml["system"]["imu_qos_reliable"].as<bool>() : true;
        const bool lidar_qos_reliable =
            yaml["system"]["lidar_qos_reliable"] ? yaml["system"]["lidar_qos_reliable"].as<bool>() : true;

        rclcpp::QoS imu_qos{rclcpp::KeepLast(static_cast<std::size_t>(imu_qos_depth))};
        rclcpp::QoS lidar_qos{rclcpp::KeepLast(static_cast<std::size_t>(lidar_qos_depth))};
        if (imu_qos_reliable) {
            imu_qos.reliable();
        } else {
            imu_qos.best_effort();
        }
        if (lidar_qos_reliable) {
            lidar_qos.reliable();
        } else {
            lidar_qos.best_effort();
        }
        LOG(INFO) << "ROS sensor QoS: IMU=" << (imu_qos_reliable ? "reliable" : "best_effort")
                  << " depth=" << imu_qos_depth << ", LiDAR="
                  << (lidar_qos_reliable ? "reliable" : "best_effort") << " depth=" << lidar_qos_depth;

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, imu_qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
                imu->orientation =
                    Quatd(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

                ProcessIMU(imu);
            });

        cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, lidar_qos,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) { ProcessLidar(std::move(cloud)); });

        livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic_, lidar_qos,
            [this](livox_ros_driver2::msg::CustomMsg::SharedPtr cloud) { ProcessLidar(std::move(cloud)); });

        if (options_.enable_lidar_rviz_) {
            std::string scan_topic = yaml["system"]["rviz_current_scan_topic"] ? yaml["system"]["rviz_current_scan_topic"].as<std::string>() : "lightning/current_scan";
            std::string map_topic = yaml["system"]["rviz_global_map_topic"] ? yaml["system"]["rviz_global_map_topic"].as<std::string>() : "lightning/global_map";
            cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(scan_topic, 10);
            map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(map_topic, 1);
        }

        if (options_.enable_path_rviz_) {
            path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("lightning/path", 10);
        }

        if (options_.pub_odom_) {
            odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("lightning/odom", 10);
            nav_state_pub_ = node_->create_publisher<msg::NavState>("lightning/nav_state", 10);
        }

        if (options_.pub_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        }

        savemap_service_ = node_->create_service<SaveMapService>(
            "lightning/save_map", [this](const SaveMapService::Request::SharedPtr req,
                                         SaveMapService::Response::SharedPtr res) { SaveMap(req, res); });

        savepath_service_ = node_->create_service<srv::SavePath>(
            "lightning/save_path", [this](const srv::SavePath::Request::SharedPtr req,
                                          srv::SavePath::Response::SharedPtr res) { SavePath(req, res); });

        LOG(INFO) << "SavePath service has been created.";
    }

    return true;
}

SlamSystem::~SlamSystem() {
    StopSensorWorker();

    // Loop closing owns a callback into G2P5, so its worker must stop while
    // the grid mapper is still alive.
    lc_.reset();
    g2p5_.reset();

    if (ui_) {
        ui_->Quit();
    }
}

void SlamSystem::StartSLAM(std::string map_name) {
    if (!map_name.empty()) {
        map_name_ = map_name;
    }

    if (options_.online_mode_ && options_.async_lio_ && !sensor_worker_.joinable()) {
        {
            std::lock_guard<std::mutex> lock(sensor_event_mutex_);
            stop_sensor_worker_ = false;
            accepting_sensor_events_ = true;
        }
        sensor_worker_ = std::thread([this]() { SensorWorkerLoop(); });
    }
    running_.store(true, std::memory_order_release);
}

void SlamSystem::SaveMap(const SaveMapService::Request::SharedPtr request,
                         SaveMapService::Response::SharedPtr response) {
    map_name_ = request->map_id;
    std::string save_path = "./data/" + map_name_ + "/";

    SaveMap(save_path);
    response->response = 0;
}

void SlamSystem::SaveMap(const std::string& path) {
    WaitForSensorEvents();
    std::lock_guard<std::mutex> lio_lock(lio_processing_mutex_);
    lio_->FlushPendingPointClouds();
    ProcessReadyLidarFramesLocked();

    const auto keyframes = lio_->GetAllKeyframes();
    if (keyframes.empty()) {
        LOG(WARNING) << "skip map save: LIO has not produced a keyframe";
        return;
    }

    std::string save_path = path;
    if (save_path.empty()) {
        save_path = "./data/" + map_name_ + "/";
    }

    LOG(INFO) << "slam map saving to " << save_path;

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    // auto global_map_no_loop = lio_->GetGlobalMap(true);
    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    SE3 start_pose = keyframes.front()->GetOptPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_no_loop.pcd", *global_map_no_loop);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_raw.pcd", *global_map_raw);

    if (options_.with_gridmap_) {
        auto grid_map = g2p5_ ? g2p5_->GetNewestMap() : nullptr;
        if (!grid_map) {
            LOG(WARNING) << "skip 2D grid map save: G2P5 has not produced a map";
        } else {
            /// 存为ROS兼容的模式
            auto map = grid_map->ToROS();
            const int width = map.info.width;
            const int height = map.info.height;

            cv::Mat nav_image(height, width, CV_8UC1);
            for (int y = 0; y < height; ++y) {
                const int rowStartIndex = y * width;
                for (int x = 0; x < width; ++x) {
                    const int index = rowStartIndex + x;
                    int8_t data = map.data[index];
                    if (data == 0) {                                   // Free
                        nav_image.at<uchar>(height - 1 - y, x) = 255;  // White
                    } else if (data == 100) {                          // Occupied
                        nav_image.at<uchar>(height - 1 - y, x) = 0;    // Black
                    } else {                                           // Unknown
                        nav_image.at<uchar>(height - 1 - y, x) = 128;  // Gray
                    }
                }
            }

            cv::imwrite(save_path + "/map.pgm", nav_image);

            /// yaml
            std::ofstream yamlFile(save_path + "/map.yaml");
            if (!yamlFile.is_open()) {
                LOG(ERROR) << "failed to write map.yaml";
                return;  // 文件打开失败
            }

            try {
                YAML::Emitter emitter;
                emitter << YAML::BeginMap;
                emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
                emitter << YAML::Key << "mode" << YAML::Value << "trinary";
                emitter << YAML::Key << "width" << YAML::Value << map.info.width;
                emitter << YAML::Key << "height" << YAML::Value << map.info.height;
                emitter << YAML::Key << "resolution" << YAML::Value << float(0.05);
                std::vector<double> orig{map.info.origin.position.x, map.info.origin.position.y, 0};
                emitter << YAML::Key << "origin" << YAML::Value << orig;
                emitter << YAML::Key << "negate" << YAML::Value << 0;
                emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
                emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;

                emitter << YAML::EndMap;

                yamlFile << emitter.c_str();
                yamlFile.close();
            } catch (...) {
                yamlFile.close();
                return;
            }
        }
    }

    LOG(INFO) << "map saved";
}

// ros服务回调，保存轨迹
// 如果请求里没有指定路径，则默认保存在./data/下，文件名为path_年月日时分秒.txt
// ros2 service call /lightning/save_path lightning/srv/SavePath
void SlamSystem::SavePath(const srv::SavePath::Request::SharedPtr request, srv::SavePath::Response::SharedPtr response) {
    std::string save_path = request->file_path;
    bool success = SavePath(save_path);
    std::size_t path_size = 0;
    {
        std::lock_guard<std::mutex> lio_lock(lio_processing_mutex_);
        path_size = path_.poses.size();
    }
    response->success = success;
    if (success) {
        response->message = "Path saved successfully. Total poses: " + std::to_string(path_size);
    } else {
        response->message = "Failed to save path.";
    }
}

bool SlamSystem::SavePath(const std::string& path) {
    WaitForSensorEvents();
    std::lock_guard<std::mutex> lio_lock(lio_processing_mutex_);

    std::string save_path = path;
    if (save_path.empty()) {
        char time_str[64];
        time_t now = time(nullptr);
        strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", localtime(&now));
        save_path = "./data/path_" + std::string(time_str) + ".txt";
    }

    std::ofstream file(save_path);
    if (!file.is_open()) {
        LOG(ERROR) << "Failed to open file: " << save_path;
        return false;
    }
    file<<"timestamp px py pz qx qy qz qw\n";
    for (const auto& pose : path_.poses) {
        file << std::fixed << std::setprecision(5) 
             << pose.header.stamp.sec << "." << std::setfill('0') << std::setw(9) << pose.header.stamp.nanosec << " "
             << pose.pose.position.x << " " << pose.pose.position.y << " " << pose.pose.position.z << " "
             << pose.pose.orientation.x << " " << pose.pose.orientation.y << " " 
             << pose.pose.orientation.z << " " << pose.pose.orientation.w << "\n";
    }

    file.close();
    LOG(INFO) << "Path saved to " << save_path << ". Total poses: " << path_.poses.size();
    return true;
}


void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (!running_.load(std::memory_order_acquire) || !imu) {
        return;
    }

    if (options_.online_mode_ && options_.async_lio_) {
        EnqueueSensorEvent(imu);
        return;
    }

    std::lock_guard<std::mutex> lock(lio_processing_mutex_);
    ProcessSensorEventLocked(imu);
    ProcessReadyLidarFramesLocked();
}

void SlamSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    if (!running_.load(std::memory_order_acquire) || !cloud) {
        return;
    }

    if (options_.online_mode_ && options_.async_lio_) {
        EnqueueSensorEvent(cloud);
        return;
    }

    std::lock_guard<std::mutex> lock(lio_processing_mutex_);
    ProcessSensorEventLocked(cloud);
    ProcessReadyLidarFramesLocked();
}

void SlamSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud) {
    if (!running_.load(std::memory_order_acquire) || !cloud) {
        return;
    }

    if (options_.online_mode_ && options_.async_lio_) {
        EnqueueSensorEvent(cloud);
        return;
    }

    std::lock_guard<std::mutex> lock(lio_processing_mutex_);
    ProcessSensorEventLocked(cloud);
    ProcessReadyLidarFramesLocked();
}

void SlamSystem::EnqueueSensorEvent(SensorEventPayload payload) {
    const bool is_imu = std::holds_alternative<lightning::IMUPtr>(payload);

    {
        std::lock_guard<std::mutex> lock(sensor_event_mutex_);
        if (!accepting_sensor_events_) {
            return;
        }

        if (is_imu && pending_imu_events_ >= options_.async_lio_max_pending_imu_) {
            const auto it = std::find_if(sensor_events_.begin(), sensor_events_.end(), [](const SensorEvent& event) {
                return std::holds_alternative<lightning::IMUPtr>(event.payload);
            });
            if (it != sensor_events_.end()) {
                sensor_events_.erase(it);
                --pending_imu_events_;
                ++dropped_imu_events_;
                if (dropped_imu_events_ == 1 || dropped_imu_events_ % 1000 == 0) {
                    LOG(WARNING) << "async LIO raw IMU queue overflow, dropped oldest IMU; total dropped: "
                                 << dropped_imu_events_;
                }
            }
        } else if (!is_imu && pending_lidar_events_ >= options_.async_lio_max_pending_lidar_) {
            const auto it = std::find_if(sensor_events_.begin(), sensor_events_.end(), [](const SensorEvent& event) {
                return !std::holds_alternative<lightning::IMUPtr>(event.payload);
            });
            if (it != sensor_events_.end()) {
                sensor_events_.erase(it);
                --pending_lidar_events_;
                ++dropped_lidar_events_;
                if (dropped_lidar_events_ == 1 || dropped_lidar_events_ % 100 == 0) {
                    LOG(WARNING) << "async LIO raw LiDAR queue overflow, dropped oldest scan; total dropped: "
                                 << dropped_lidar_events_;
                }
            }
        }

        sensor_events_.push_back(SensorEvent{++next_sensor_sequence_, std::move(payload)});
        if (is_imu) {
            ++pending_imu_events_;
        } else {
            ++pending_lidar_events_;
        }
    }
    sensor_event_cv_.notify_one();
}

void SlamSystem::ProcessSensorEventLocked(const SensorEventPayload& payload) {
    if (const auto* imu = std::get_if<lightning::IMUPtr>(&payload)) {
        if (!*imu) {
            return;
        }
        if (options_.use_imu_init_ && !imu_inited_) {
            LOG(INFO) << "Auto-initializing slam pose from IMU orientation (ENU): "
                      << (*imu)->orientation.coeffs().transpose();
            lio_->SetInitPose(SE3((*imu)->orientation, Vec3d::Zero()));
            imu_inited_ = true;
        }
        lio_->ProcessIMU(*imu);
        return;
    }

    if (const auto* cloud = std::get_if<sensor_msgs::msg::PointCloud2::SharedPtr>(&payload)) {
        if (*cloud) {
            lio_->ProcessPointCloud2(*cloud);
        }
        return;
    }

    const auto* cloud = std::get_if<livox_ros_driver2::msg::CustomMsg::SharedPtr>(&payload);
    if (cloud != nullptr && *cloud) {
        lio_->ProcessPointCloud2(*cloud);
    }
}

void SlamSystem::ProcessReadyLidarFramesLocked() {
    while (lio_->HasPendingLidar()) {
        const auto status = lio_->RunOnce();
        if (status == LaserMapping::RunStatus::NO_READY_SCAN) {
            break;
        }
        if (status == LaserMapping::RunStatus::OUTPUT_READY) {
            HandleLidarResultLocked();
        }
    }
}

void SlamSystem::SensorWorkerLoop() {
    constexpr std::size_t kMaxBatchSize = 256;
    while (true) {
        std::deque<SensorEvent> batch;
        {
            std::unique_lock<std::mutex> lock(sensor_event_mutex_);
            sensor_event_cv_.wait(lock, [this]() { return stop_sensor_worker_ || !sensor_events_.empty(); });
            if (sensor_events_.empty() && stop_sensor_worker_) {
                break;
            }

            while (!sensor_events_.empty() && batch.size() < kMaxBatchSize) {
                SensorEvent event = std::move(sensor_events_.front());
                sensor_events_.pop_front();
                if (std::holds_alternative<lightning::IMUPtr>(event.payload)) {
                    --pending_imu_events_;
                } else {
                    --pending_lidar_events_;
                }
                batch.emplace_back(std::move(event));
            }
        }

        {
            std::lock_guard<std::mutex> lock(lio_processing_mutex_);
            for (const auto& event : batch) {
                ProcessSensorEventLocked(event.payload);
                // Retry after every IMU as well as every LiDAR. This preserves
                // event order without transferring a large raw backlog into
                // LaserMapping's much smaller synchronization buffer.
                ProcessReadyLidarFramesLocked();
            }
        }

        {
            std::lock_guard<std::mutex> lock(sensor_event_mutex_);
            completed_sensor_sequence_ = std::max(completed_sensor_sequence_, batch.back().sequence);
        }
        sensor_event_drained_cv_.notify_all();
    }

    sensor_event_drained_cv_.notify_all();
}

void SlamSystem::WaitForSensorEvents() {
    if (!options_.online_mode_ || !options_.async_lio_ || !sensor_worker_.joinable()) {
        return;
    }

    std::unique_lock<std::mutex> lock(sensor_event_mutex_);
    const std::uint64_t target_sequence = next_sensor_sequence_;
    sensor_event_drained_cv_.wait(lock,
                                  [this, target_sequence]() { return completed_sensor_sequence_ >= target_sequence; });
}

void SlamSystem::StopSensorWorker() {
    running_.store(false, std::memory_order_release);

    {
        std::lock_guard<std::mutex> lock(sensor_event_mutex_);
        accepting_sensor_events_ = false;
        stop_sensor_worker_ = true;
    }
    sensor_event_cv_.notify_one();

    if (sensor_worker_.joinable()) {
        sensor_worker_.join();
    }

    if (dropped_lidar_events_ > 0 || dropped_imu_events_ > 0) {
        LOG(WARNING) << "async LIO event drops: LiDAR=" << dropped_lidar_events_
                     << ", IMU=" << dropped_imu_events_;
    }
}

void SlamSystem::HandleLidarResultLocked() {
    const auto state = lio_->GetState();
    const auto processed_stamp = math::FromSec(state.timestamp_);
    const bool can_publish_ros = !options_.online_mode_ || rclcpp::ok();

    if (options_.log_pose_opt_) {
        auto q = state.rot_.unit_quaternion();

        char log_buf[256];
        snprintf(log_buf, sizeof(log_buf),
                "lio pose: [%.4f, %.4f, %.4f]\tq: [%.4f, %.4f, %.4f, %.4f]\tvelocity: [%.3f, %.3f, %.3f]m/s",
                state.pos_.x(), state.pos_.y(), state.pos_.z(),
                q.x(), q.y(), q.z(), q.w(),
                state.vel_.x(), state.vel_.y(), state.vel_.z());
        LOG(INFO) << log_buf;
    }

    static uint8_t count = 0;
    if (nav_state_pub_ != nullptr) {
        // 真正的回环后的位姿，而非EKF的里程计位姿
        SE3 pose_opt = lio_->GetOptPose();
        auto q_opt = pose_opt.unit_quaternion();

        msg::NavState ns;
        ns.header.stamp = processed_stamp;
        ns.header.frame_id = "map";
        ns.pose.position.x = pose_opt.translation().x();
        ns.pose.position.y = pose_opt.translation().y();
        ns.pose.position.z = pose_opt.translation().z();
        ns.pose.orientation.x = q_opt.x();
        ns.pose.orientation.y = q_opt.y();
        ns.pose.orientation.z = q_opt.z();
        ns.pose.orientation.w = q_opt.w();
        ns.velocity.x = state.vel_.x();
        ns.velocity.y = state.vel_.y();
        ns.velocity.z = state.vel_.z();
        ns.confidence = 1.0;
        ns.pose_is_ok = true;
        if (can_publish_ros) {
            nav_state_pub_->publish(ns);
        }

        if (tf_broadcaster_ != nullptr && can_publish_ros) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header = ns.header;
            tf_msg.child_frame_id = "lidar_link";
            tf_msg.transform.translation.x = ns.pose.position.x;
            tf_msg.transform.translation.y = ns.pose.position.y;
            tf_msg.transform.translation.z = ns.pose.position.z;
            tf_msg.transform.rotation = ns.pose.orientation;
            tf_broadcaster_->sendTransform(tf_msg);
        }

        if (odom_pub_ != nullptr && can_publish_ros) {
            nav_msgs::msg::Odometry odom;
            odom.header = ns.header;
            odom.child_frame_id = "lidar_link";
            odom.pose.pose = ns.pose;
            odom.twist.twist.linear = ns.velocity;
            odom_pub_->publish(odom);
        }

        if (options_.enable_path_rviz_ && path_pub_ != nullptr) {
            geometry_msgs::msg::PoseStamped ps;
            if (count < 10) {
                LOG(INFO) << "publishing path pose: ["
                          << ns.pose.position.x << ", " << ns.pose.position.y << ", " << ns.pose.position.z << "]";
                count++;
            }
            ps.header = ns.header;
            ps.pose = ns.pose;
            path_.header = ns.header;
            path_.poses.push_back(ps);
            if (can_publish_ros) {
                path_pub_->publish(path_);
            }
        }
    } else if (options_.enable_path_rviz_) {
        // 离线模式或未启用 pub_odom 时，手动维护 path_
        SE3 pose_opt = lio_->GetOptPose();
        auto q_opt = pose_opt.unit_quaternion();

        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = processed_stamp;
        ps.header.frame_id = "map";
        ps.pose.position.x = pose_opt.translation().x();
        ps.pose.position.y = pose_opt.translation().y();
        ps.pose.position.z = pose_opt.translation().z();
        ps.pose.orientation.x = q_opt.x();
        ps.pose.orientation.y = q_opt.y();
        ps.pose.orientation.z = q_opt.z();
        ps.pose.orientation.w = q_opt.w();

        if (count < 10) {
            LOG(INFO) << "recording path pose (no pub): [" << ps.pose.position.x << ", " << ps.pose.position.y
                      << ", " << ps.pose.position.z << "]";
            count++;
        }

        path_.header = ps.header;
        path_.poses.push_back(ps);
    }

    if (options_.enable_lidar_rviz_ && cloud_pub_ != nullptr && can_publish_ros) {
        auto scan_world = lio_->GetScanDownWorld();
        if (scan_world && !scan_world->empty()) {
            sensor_msgs::msg::PointCloud2 scan_msg;
            pcl::toROSMsg(*scan_world, scan_msg);
            scan_msg.header.frame_id = "map";
            scan_msg.header.stamp = processed_stamp;
            cloud_pub_->publish(scan_msg);
        }
    }

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }

    if (map_pub_ != nullptr && can_publish_ros) {
        static int kf_count = 0;
        if (kf_count++ % 3 == 0) { // 每3个关键帧发布一个全局地图
            auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
            sensor_msgs::msg::PointCloud2 ros_map;
            pcl::toROSMsg(*global_map, ros_map);
            ros_map.header.frame_id = "map";
            ros_map.header.stamp = processed_stamp;
            map_pub_->publish(ros_map);
        }
    }
}

void SlamSystem::Spin() {
    if (options_.online_mode_ && node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning
