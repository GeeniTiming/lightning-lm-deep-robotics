//
// Created by xiang on 25-9-12.
//

#include <filesystem>
#include <fstream>
#include <iomanip>
#include "core/system/loc_system.h"
#include "core/lightning_math.hpp"
#include "core/localization/localization.h"
#include "core/localization/lidar_loc/lidar_loc.h"
#include <pcl/common/transforms.h>
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"

namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() { loc_->Finish(); }

bool LocSystem::Init(const std::string &yaml_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);
    std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    options_.pub_tf_ = yaml.GetValue<bool>("system", "pub_tf", true);
    options_.pub_odom_ = yaml.GetValue<bool>("system", "pub_odom", true);
    options_.log_pose_opt_ = yaml.GetValue<bool>("system", "log_pose_opt", false);
    options_.use_imu_init_ = yaml.GetValue<bool>("system", "use_imu_orient", false);

    // imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic", "/IMU");
    // cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic", "/LIDAR/POINTS");
    // livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic", "/livox/lidar");
    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");

    options_.use_init_pose_ = yaml.GetValue<bool>("system", "use_init_pose", false);
    if (options_.use_init_pose_) {
        auto &node = yaml.yaml_node();
        auto init_pos = node["system"]["init_pos"].as<std::vector<double>>();
        auto init_quat = node["system"]["init_quat"].as<std::vector<double>>();
        LOG(INFO) << "init_pos: " << init_pos.size() << ", init_quat: " << init_quat.size();
        if (init_pos.size() == 3 && init_quat.size() == 4) {
            options_.init_pose_ = SE3(Quatd(init_quat[3], init_quat[0], init_quat[1], init_quat[2]),
                                     Vec3d(init_pos[0], init_pos[1], init_pos[2]));
            LOG(INFO) << "set init pose from yaml: " << options_.init_pose_.translation().transpose();
        } else {
            LOG(WARNING) << "init_pos or init_quat size mismatch, init_pos: " << init_pos.size()
                         << ", init_quat: " << init_quat.size();
        }
    }

    // std::string map_path = yaml.GetValue<std::string>("system", "map_path", "./data/new_map/");

    LOG(INFO) << "online mode, creating ros2 node ... ";

    /// subscribers
    node_ = std::make_shared<rclcpp::Node>("lightning_slam");

    rclcpp::QoS qos(10);

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

            imu->orientation = Quatd(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

            ProcessIMU(imu);
        });

    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    if (options_.pub_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    }

    if (options_.pub_odom_) {
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("lightning/odom", 10);
        nav_state_pub_ = node_->create_publisher<msg::NavState>("lightning/nav_state", 10);
    }

    if (yaml.GetValue<bool>("system", "enable_lidar_loc_rviz", false)) {
        std::string scan_topic = yaml.GetValue<std::string>("system", "rviz_current_scan_topic", "lightning/current_scan_cloud");
        cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(scan_topic, 10);
        global_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lightning/global_map", 1);
    }
    if(yaml.GetValue<bool>("system", "enable_path_rviz", false)) {
        path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("lightning/path", 10);
    }

    savepath_service_ = node_->create_service<srv::SavePath>(
        "lightning/save_path", [this](const srv::SavePath::Request::SharedPtr req,
                                      srv::SavePath::Response::SharedPtr res) { SavePath(req, res); });

    bool ret = loc_->Init(yaml_path, map_path);
    if (ret) {
        loc_->SetLocalizationResultCallback(
            [this](const loc::LocalizationResult& result) { PublishLocalizationResult(result); });
        LOG(INFO) << "online loc node has been created.";
    }

    return ret;
}

void LocSystem::Start() {
    SetInitPose(options_.use_init_pose_ ? options_.init_pose_ : SE3());
}

void LocSystem::SetInitPose(const SE3 &pose) {
    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();

    loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
    loc_started_ = true;
}

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    if (loc_started_) {
        loc_->ProcessIMUMsg(imu);
    } else if (options_.use_imu_init_) {
        // 如果没有收到初始位姿，且开启了使用 IMU 的 orientation 进行初始化
        // 假设初始位置为原点 (0,0,0)
        LOG(INFO) << "Auto-initializing pose from IMU orientation (ENU): "
                  << imu->orientation.coeffs().transpose();
        SetInitPose(SE3(imu->orientation, Vec3d::Zero()));
    }
}

void LocSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLidarMsg(cloud);
        PublishDebugClouds(cloud->header.stamp);
    }
}

void LocSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLivoxLidarMsg(cloud);
        PublishDebugClouds(cloud->header.stamp);
    }
}

void LocSystem::PublishLocalizationResult(const loc::LocalizationResult& result) {
    const auto stamp = math::FromSec(result.timestamp_);
    const auto q = result.pose_.unit_quaternion();
    const Vec3d velocity = result.pose_.so3() * result.vel_b_;

    geometry_msgs::msg::Pose pose;
    pose.position.x = result.pose_.translation().x();
    pose.position.y = result.pose_.translation().y();
    pose.position.z = result.pose_.translation().z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    if (options_.log_pose_opt_) {
        LOG(INFO) << "fused pose: " << result.pose_.translation().transpose()
                  << ", confidence: " << result.confidence_;
    }

    msg::NavState ns;
    ns.header.stamp = stamp;
    ns.header.frame_id = "map";
    ns.pose = pose;
    ns.velocity.x = velocity.x();
    ns.velocity.y = velocity.y();
    ns.velocity.z = velocity.z();
    ns.confidence = result.confidence_;
    ns.pose_is_ok = result.valid_;

    if (nav_state_pub_) {
        nav_state_pub_->publish(ns);
    }
    if (odom_pub_) {
        nav_msgs::msg::Odometry odom;
        odom.header = ns.header;
        odom.child_frame_id = "lidar_link";
        odom.pose.pose = pose;
        odom.twist.twist.linear = ns.velocity;
        odom_pub_->publish(odom);
    }
    if (tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header = ns.header;
        tf_msg.child_frame_id = "lidar_link";
        tf_msg.transform.translation.x = pose.position.x;
        tf_msg.transform.translation.y = pose.position.y;
        tf_msg.transform.translation.z = pose.position.z;
        tf_msg.transform.rotation = pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);
    }

    geometry_msgs::msg::PoseStamped ps;
    ps.header = ns.header;
    ps.pose = pose;
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        if (trajectory_.empty() || result.timestamp_ > ToSec(trajectory_.back().header.stamp) + 1e-7) {
            trajectory_.push_back(ps);
        } else if (std::abs(result.timestamp_ - ToSec(trajectory_.back().header.stamp)) <= 1e-7) {
            trajectory_.back() = ps;
        }

        if (path_pub_ && (path_.poses.empty() || result.timestamp_ - last_path_pub_time_ >= 0.05)) {
            path_.header = ps.header;
            path_.poses.push_back(ps);
            last_path_pub_time_ = result.timestamp_;
            path_pub_->publish(path_);
        }
    }
}

void LocSystem::PublishDebugClouds(const builtin_interfaces::msg::Time& stamp) {
    if (cloud_pub_) {
        auto scan_world = loc_->GetLIO()->GetScanDownWorld();
        if (scan_world && !scan_world->empty()) {
            CloudPtr scan_map(new PointCloudType);
            pcl::transformPointCloud(*scan_world, *scan_map, loc_->GetMapFromOdom().matrix());
            sensor_msgs::msg::PointCloud2 scan_msg;
            pcl::toROSMsg(*scan_map, scan_msg);
            scan_msg.header.frame_id = "map";
            scan_msg.header.stamp = stamp;
            cloud_pub_->publish(scan_msg);
        }
    }

    const double current_time = ToSec(stamp);
    if (global_map_pub_ && current_time - last_map_pub_time_ > 10.0) {
        CloudPtr global_map = loc_->GetLidarLoc()->GetMap()->GetStaticCloud2();
        if (global_map && !global_map->empty()) {
            sensor_msgs::msg::PointCloud2 map_msg;
            pcl::toROSMsg(*global_map, map_msg);
            map_msg.header.frame_id = "map";
            map_msg.header.stamp = stamp;
            global_map_pub_->publish(map_msg);
            last_map_pub_time_ = current_time;
        }
    }
}

void LocSystem::Spin() {
    if (node_ != nullptr) {
        spin(node_);
    }
}

// ros服务回调，保存轨迹
// 如果请求里没有指定路径，则默认保存在./data/下，文件名为path_年月日时分秒.txt
// ros2 service call /lightning/save_path lightning/srv/SavePath "{file_path: 'data/traj.txt'}"
void LocSystem::SavePath(const srv::SavePath::Request::SharedPtr request, srv::SavePath::Response::SharedPtr response) {
    loc_->FlushPendingLidar();

    std::string save_path = request->file_path;
    if (save_path.empty()) {
        char time_str[64];
        time_t now = time(nullptr);
        strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", localtime(&now));
        save_path = "./data/path_" + std::string(time_str) + ".txt";
    }

    std::ofstream file(save_path);
    if (!file.is_open()) {
        response->success = false;
        response->message = "Failed to open file: " + save_path;
        return;
    }

    std::vector<geometry_msgs::msg::PoseStamped> trajectory;
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        trajectory = trajectory_;
    }

    file << "timestamp px py pz qx qy qz qw\n";
    for (const auto& pose : trajectory) {
        file << std::fixed << std::setprecision(5) 
             << pose.header.stamp.sec << "." << std::setfill('0') << std::setw(9) << pose.header.stamp.nanosec << " "
             << pose.pose.position.x << " " << pose.pose.position.y << " " << pose.pose.position.z << " "
             << pose.pose.orientation.x << " " << pose.pose.orientation.y << " " 
             << pose.pose.orientation.z << " " << pose.pose.orientation.w << "\n";
    }

    file.close();
    response->success = true;
    response->message =
        "Fused path saved successfully to " + save_path + ". Total poses: " + std::to_string(trajectory.size());
}

}  // namespace lightning
