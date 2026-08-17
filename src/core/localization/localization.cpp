#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "core/localization/lidar_loc/lidar_loc.h"
#include "core/localization/localization.h"
#include "core/localization/pose_graph/pgo.h"
#include "io/yaml_io.h"
#include "ui/pangolin_window.h"

namespace lightning::loc {

// ！ 构造函数
Localization::Localization(Options options) { options_ = options; }

// ！初始化函数
bool Localization::Init(const std::string& yaml_path, const std::string& global_map_path) {
    UL lock(global_mutex_);
    if (lidar_loc_ != nullptr) {
        // 若已经启动，则变为初始化
        Finish();
    }

    YAML_IO yaml(yaml_path);
    options_.with_ui_ = yaml.GetValue<bool>("system", "with_ui");

    /// lidar odom前端
    LaserMapping::Options opt_lio;
    opt_lio.is_in_slam_mode_ = false;

    lio_ = std::make_shared<LaserMapping>(opt_lio);
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio";
        return false;
    }

    /// 激光定位
    LidarLoc::Options lidar_loc_options;
    lidar_loc_options.update_dynamic_cloud_ = yaml.GetValue<bool>("lidar_loc", "update_dynamic_cloud");
    lidar_loc_options.force_2d_ = yaml.GetValue<bool>("lidar_loc", "force_2d");
    lidar_loc_options.map_option_.enable_dynamic_polygon_ = false;
    lidar_loc_options.map_option_.map_path_ = global_map_path;
    lidar_loc_ = std::make_shared<LidarLoc>(lidar_loc_options);

    if (options_.with_ui_) {
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->SetCurrentScanSize(10);
        ui_->Init();

        lidar_loc_->SetUI(ui_);

        // lio_->SetUI(ui_);
    }

    lidar_loc_->Init(yaml_path);

    /// pose graph
    pgo_ = std::make_shared<PGO>();
    pgo_->SetDebug(false);

    ///  各模块的异步调用
    options_.enable_lidar_loc_skip_ = yaml.GetValue<bool>("system", "enable_lidar_loc_skip");
    options_.enable_lidar_loc_rviz_ = yaml.GetValue<bool>("system", "enable_lidar_loc_rviz");
    options_.lidar_loc_skip_num_ = yaml.GetValue<int>("system", "lidar_loc_skip_num");
    options_.enable_lidar_odom_skip_ = yaml.GetValue<bool>("system", "enable_lidar_odom_skip");
    options_.lidar_odom_skip_num_ = yaml.GetValue<int>("system", "lidar_odom_skip_num");
    options_.loc_on_kf_ = yaml.GetValue<bool>("lidar_loc", "loc_on_kf");
    options_.async_lidar_loc_ = yaml.GetValue<bool>("lidar_loc", "async_lidar_loc", false);
    options_.map_odom_correction_gain_ =
        yaml.GetValue<double>("lidar_loc", "map_odom_correction_gain", 0.05);
    options_.map_odom_max_translation_jump_ =
        yaml.GetValue<double>("lidar_loc", "map_odom_max_translation_jump", 0.5);
    options_.map_odom_max_rotation_jump_deg_ =
        yaml.GetValue<double>("lidar_loc", "map_odom_max_rotation_jump_deg", 15.0);

    lidar_odom_proc_cloud_.SetMaxSize(1);
    lidar_loc_proc_cloud_.SetMaxSize(1);

    lidar_odom_proc_cloud_.SetName("激光里程计");
    lidar_loc_proc_cloud_.SetName("激光定位");

    // 允许跳帧
    lidar_loc_proc_cloud_.SetSkipParam(options_.enable_lidar_loc_skip_, options_.lidar_loc_skip_num_);
    lidar_odom_proc_cloud_.SetSkipParam(options_.enable_lidar_odom_skip_, options_.lidar_odom_skip_num_);

    lidar_odom_proc_cloud_.SetProcFunc([this](CloudPtr cloud) { LidarOdomProcCloud(cloud); });
    lidar_loc_proc_cloud_.SetProcFunc([this](CloudPtr cloud) { LidarLocProcCloud(cloud); });

    if (options_.online_mode_) {
        lidar_odom_proc_cloud_.Start();
        if (options_.async_lidar_loc_) {
            lidar_loc_proc_cloud_.Start();
        }
    }

    return true;
}

void Localization::ProcessLidarMsg(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    UL lock(global_mutex_);
    if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr) {
        return;
    }

    // Keep the original ROS message until LaserMapping has inspected the
    // RoboSense header/point timestamps.  Converting to CloudPtr here used to
    // bypass its publish-time detection and physical-scan rebatching.
    lio_->ProcessPointCloud2(cloud);
    if (options_.online_mode_) {
        ScheduleLidarOdomDrain();
    } else {
        DrainLidarOdom();
    }
}

void Localization::ProcessLivoxLidarMsg(const livox_ros_driver2::msg::CustomMsg::SharedPtr cloud) {
    UL lock(global_mutex_);
    if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    if (options_.online_mode_) {
        ScheduleLidarOdomDrain();
    } else {
        DrainLidarOdom();
    }
}

void Localization::LidarOdomProcCloud(CloudPtr cloud) {
    if (lio_ == nullptr) {
        return;
    }

    // CloudPtr is retained for callers that already provide preprocessed
    // clouds. Online ROS input now queues only a nullptr wake-up token; the
    // actual scans live in LaserMapping's timestamp-aware buffer.
    if (cloud) {
        lio_->ProcessPointCloud2(cloud);
    }
    DrainLidarOdom();

    lio_drain_scheduled_.store(false, std::memory_order_release);
    if (lio_->ShouldProcessLidar()) {
        ScheduleLidarOdomDrain();
    }
}

void Localization::ScheduleLidarOdomDrain() {
    bool expected = false;
    if (lio_drain_scheduled_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        lidar_odom_proc_cloud_.AddMessage(nullptr);
    }
}

void Localization::DrainLidarOdom() {
    std::lock_guard<std::mutex> lock(lio_processing_mutex_);
    while (true) {
        const auto status = lio_->RunOnce();
        if (status == LaserMapping::RunStatus::NO_READY_SCAN) {
            break;
        }
        if (status == LaserMapping::RunStatus::FRAME_CONSUMED) {
            continue;
        }
        HandleLidarOdomOutput();
    }
}

void Localization::HandleLidarOdomOutput() {

    auto lo_state = lio_->GetState();

    lidar_loc_->ProcessLO(lo_state);
    pgo_->ProcessLidarOdom(lo_state);
    PublishCorrectedLidarOdom(lo_state);

    // LOG(INFO) << "LO pose: " << std::setprecision(12) << lo_state.timestamp_ << " "
    //           << lo_state.GetPose().translation().transpose();

    /// 获得lio的关键帧
    if (options_.loc_on_kf_) {
        auto kf = lio_->GetKeyframe();
        if (kf == lio_kf_) {
            /// 关键帧未更新，那就只更新IMU状态

            // auto dr_state = lio_->GetState();
            // lidar_loc_->ProcessDR(dr_state);
            // pgo_->ProcessDR(dr_state);
            return;
        }

        lio_kf_ = kf;

        auto scan = lio_->GetScanUndist();
        if (!scan || scan->empty()) {
            return;
        }
        const double scan_duration = scan->points.back().timestamp * 1e-3;
        const double scan_start_time = lo_state.timestamp_ - scan_duration;
        scan->header.stamp = static_cast<std::uint64_t>(std::llround(scan_start_time * 1e9));

        if (options_.online_mode_ && options_.async_lidar_loc_) {
            lidar_loc_proc_cloud_.AddMessage(scan);
        } else {
            LidarLocProcCloud(scan);
        }
    } else {
        auto scan = lio_->GetScanUndist();
        if (!scan || scan->empty()) {
            return;
        }
        const double scan_duration = scan->points.back().timestamp * 1e-3;
        const double scan_start_time = lo_state.timestamp_ - scan_duration;
        scan->header.stamp = static_cast<std::uint64_t>(std::llround(scan_start_time * 1e9));

        if (options_.online_mode_ && options_.async_lidar_loc_) {
            lidar_loc_proc_cloud_.AddMessage(scan);
        } else {
            LidarLocProcCloud(scan);
        }
    }
}

void Localization::LidarLocProcCloud(CloudPtr scan_undist) {
    lidar_loc_->ProcessCloud(scan_undist);

    auto res = lidar_loc_->GetLocalizationResult();
    UpdateMapFromOdom(res);
    pgo_->ProcessLidarLoc(res);

    if (ui_) {
        // Twi with Til, here pose means Twl, thus Til=I
        ui_->UpdateScan(scan_undist, res.pose_);
    }

    if (loc_state_callback_) {
        auto loc_state = std::make_shared<std_msgs::msg::Int32>();
        loc_state->data = static_cast<int>(res.status_);
        LOG(INFO) << "loc_state: " << loc_state->data;
        loc_state_callback_(*loc_state);
    }
}

void Localization::ProcessIMUMsg(IMUPtr imu) {
    UL lock(global_mutex_);

    if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr) {
        return;
    }

    double this_imu_time = imu->timestamp;
    if (last_imu_time_ > 0 && this_imu_time < last_imu_time_) {
        LOG(WARNING) << "IMU 时间异常：" << this_imu_time << ", last: " << last_imu_time_;
    }
    last_imu_time_ = this_imu_time;

    /// 里程计处理IMU
    lio_->ProcessIMU(imu);
    if (options_.online_mode_ && lio_->ShouldProcessLidar()) {
        ScheduleLidarOdomDrain();
    }

    // Do not publish/fuse GetIMUState() here.  In online mode the scan worker
    // deliberately processes buffered lidar asynchronously; the IMU receive
    // time may therefore be seconds ahead of the last corrected LIO state.
    // The stable scan-corrected state is forwarded from
    // HandleLidarOdomOutput().
}

// void Localization::ProcessOdomMsg(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
//     UL lock(global_mutex_);
//
//     if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr) {
//         return;
//     }
//     double this_odom_time = ToSec(odom_msg->header.stamp);
//     if (last_odom_time_ > 0 && this_odom_time < last_odom_time_) {
//         LOG(WARNING) << "Odom Time Abnormal:" << this_odom_time << ", last: " << last_odom_time_;
//     }
//     last_odom_time_ = this_odom_time;
//
//     lio_->ProcessOdometry(odom_msg);
//
//     if (!lio_->GetbOdomHF()) {
//         return;
//     }
//
//     auto dr_state = lio_->GetStateHF(mapping::FasterLioMapping::kHFStateOdomFiltered);
//
//     constexpr auto kThVbrbStill = 0.03;  // 0.08;
//     constexpr auto kThOmegaStill = 0.03;
//     if (dr_state.Getvwi().norm() < kThVbrbStill && dr_state.Getwii().norm() < kThOmegaStill) {
//         dr_state.is_parking_ = true;
//         dr_state.Setvwi(Vec3d::Zero());
//         dr_state.Setwii(Vec3d::Zero());
//     }
//
//     lidar_loc_->ProcessDR(dr_state);
//     pgo_->ProcessDR(dr_state);
// }

void Localization::Finish() {
    FlushPendingLidar();
    lidar_odom_proc_cloud_.Quit();
    lidar_loc_proc_cloud_.Quit();
    lidar_loc_->Finish();
    if (ui_) {
        ui_->Quit();
    }
}

void Localization::FlushPendingLidar() {
    if (!lio_) {
        return;
    }
    lio_->FlushPendingPointClouds();
    DrainLidarOdom();
}

void Localization::SetExternalPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    UL lock(global_mutex_);
    LOG(INFO) << "Localization setting external pose: " << t.transpose() << ", " << q.coeffs().transpose();
    /// 设置外部重定位的pose
    if (lidar_loc_) {
        lidar_loc_->SetInitialPose(SE3(q, t));
    }
    if (lio_) {
        lio_->SetInitPose(SE3(q, t));
    }
    {
        std::lock_guard<std::mutex> fusion_lock(fusion_mutex_);
        lio_pose_history_.clear();
        map_from_odom_initialized_ = false;
        latest_lidar_loc_result_ = LocalizationResult();
    }
}

void Localization::UpdateMapFromOdom(const LocalizationResult& lidar_loc_result) {
    if (!lidar_loc_result.lidar_loc_valid_) {
        return;
    }

    std::lock_guard<std::mutex> lock(fusion_mutex_);
    if (map_from_odom_initialized_ &&
        (!lidar_loc_result.lidar_loc_odom_error_normal_ ||
         !lidar_loc_result.lidar_loc_smooth_flag_)) {
        return;
    }
    SE3 odom_pose_at_loc;
    NavState best_match;
    const bool interpolated = math::PoseInterp<NavState>(
        lidar_loc_result.timestamp_, lio_pose_history_,
        [](const NavState& state) { return state.timestamp_; },
        [](const NavState& state) { return state.GetPose(); },
        odom_pose_at_loc, best_match, 0.5);
    if (!interpolated) {
        LOG(WARNING) << "cannot update map->odom: no LIO pose near lidar localization time "
                     << std::setprecision(16) << lidar_loc_result.timestamp_;
        return;
    }

    const SE3 measured_map_from_odom = lidar_loc_result.pose_ * odom_pose_at_loc.inverse();
    if (!map_from_odom_initialized_) {
        map_from_odom_ = measured_map_from_odom;
        map_from_odom_initialized_ = true;
        LOG(INFO) << "initialized stable map->odom correction: "
                  << map_from_odom_.translation().transpose();
    } else {
        const SE3 delta = map_from_odom_.inverse() * measured_map_from_odom;
        const double translation_jump = delta.translation().norm();
        const double rotation_jump_deg = delta.so3().log().norm() * 180.0 / M_PI;
        if (translation_jump > options_.map_odom_max_translation_jump_ ||
            rotation_jump_deg > options_.map_odom_max_rotation_jump_deg_) {
            LOG(WARNING) << "reject map->odom correction jump: translation=" << translation_jump
                         << " m, rotation=" << rotation_jump_deg << " deg";
            return;
        }

        const double gain = std::clamp(options_.map_odom_correction_gain_, 0.0, 1.0);
        map_from_odom_ = map_from_odom_ * SE3::exp(gain * delta.log());
    }
    latest_lidar_loc_result_ = lidar_loc_result;
    latest_lidar_loc_result_.valid_ = true;
}

void Localization::PublishCorrectedLidarOdom(const NavState& lio_state) {
    LocalizationResult result;
    bool ready = false;
    {
        std::lock_guard<std::mutex> lock(fusion_mutex_);
        lio_pose_history_.push_back(lio_state);
        while (lio_pose_history_.size() > 1000) {
            lio_pose_history_.pop_front();
        }

        if (map_from_odom_initialized_) {
            result = latest_lidar_loc_result_;
            result.timestamp_ = lio_state.timestamp_;
            result.pose_ = map_from_odom_ * lio_state.GetPose();
            result.valid_ = true;
            result.status_ = LocalizationStatus::GOOD;
            result.rel_pose_set_ = true;
            result.rel_pose_ = lio_state.GetPose();
            result.vel_b_ = lio_state.GetRot().inverse() * lio_state.GetVel();
            ready = true;
        }
    }

    if (ready) {
        EmitLocalizationResult(result);
    }
}

void Localization::EmitLocalizationResult(const LocalizationResult& result) {
    loc_result_ = result;
    if (tf_callback_ && loc_result_.valid_) {
        tf_callback_(loc_result_.ToGeoMsg());
    }
    if (localization_result_callback_ && loc_result_.valid_) {
        localization_result_callback_(loc_result_);
    }
    if (ui_) {
        ui_->UpdateNavState(loc_result_.ToNavState());
        ui_->UpdateRecentPose(loc_result_.pose_);
    }
}

void Localization::SetTFCallback(Localization::TFCallback&& callback) { tf_callback_ = callback; }

void Localization::SetLocalizationResultCallback(LocalizationResultCallback&& callback) {
    localization_result_callback_ = std::move(callback);
}

}  // namespace lightning::loc
