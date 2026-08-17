//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <vector>

#include "core/localization/localization.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

#include "io/yaml_io.h"

DEFINE_string(input_bag, "", "输入数据包");
DEFINE_string(config, "./config/default.yaml", "配置文件");
DEFINE_string(map_path, "./data/new_map/", "地图路径");
DEFINE_string(output_path, "", "融合定位轨迹输出路径（TUM 文本格式）");

/// 运行定位的测试
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_input_bag.empty()) {
        LOG(ERROR) << "未指定输入数据";
        return -1;
    }

    using namespace lightning;

    RosbagIO rosbag(FLAGS_input_bag);

    loc::Localization::Options options;
    options.online_mode_ = false;

    loc::Localization loc(options);
    if (!loc.Init(FLAGS_config, FLAGS_map_path)) {
        LOG(ERROR) << "定位初始化失败";
        return -1;
    }

    std::vector<loc::LocalizationResult> trajectory;
    std::mutex trajectory_mutex;
    loc.SetLocalizationResultCallback([&trajectory, &trajectory_mutex](const loc::LocalizationResult& result) {
        std::lock_guard<std::mutex> lock(trajectory_mutex);
        if (trajectory.empty() || result.timestamp_ > trajectory.back().timestamp_ + 1e-7) {
            trajectory.push_back(result);
        } else if (std::abs(result.timestamp_ - trajectory.back().timestamp_) <= 1e-7) {
            trajectory.back() = result;
        }
    });

    lightning::YAML_IO yaml(FLAGS_config);
    std::string lidar_topic = yaml.GetValue<std::string>("common", "lidar_topic");
    std::string imu_topic = yaml.GetValue<std::string>("common", "imu_topic");

    rosbag
        .AddImuHandle(imu_topic,
                      [&loc](IMUPtr imu) {
                          loc.ProcessIMUMsg(imu);
                          usleep(1000);
                          return true;
                      })
        .AddPointCloud2Handle(lidar_topic,
                              [&loc](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                                  loc.ProcessLidarMsg(cloud);
                                  usleep(1000);
                                  return true;
                              })
        .AddLivoxCloudHandle("/livox/lidar",
                             [&loc](livox_ros_driver2::msg::CustomMsg::SharedPtr cloud) {
                                 loc.ProcessLivoxLidarMsg(cloud);
                                 usleep(1000);
                                 return true;
                             })
        .Go();

    Timer::PrintAll();
    loc.Finish();

    if (!FLAGS_output_path.empty()) {
        std::ofstream file(FLAGS_output_path);
        if (!file.is_open()) {
            LOG(ERROR) << "无法写入融合定位轨迹: " << FLAGS_output_path;
            return -1;
        }

        std::lock_guard<std::mutex> lock(trajectory_mutex);
        file << "timestamp px py pz qx qy qz qw\n";
        file << std::fixed << std::setprecision(9);
        for (const auto& result : trajectory) {
            const auto q = result.pose_.unit_quaternion();
            const auto t = result.pose_.translation();
            file << result.timestamp_ << " " << t.x() << " " << t.y() << " " << t.z() << " "
                 << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        }
        LOG(INFO) << "Fused path saved to " << FLAGS_output_path
                  << ". Total poses: " << trajectory.size();
    }

    LOG(INFO) << "done";

    return 0;
}
