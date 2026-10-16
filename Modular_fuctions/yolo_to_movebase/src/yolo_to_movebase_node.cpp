#include "yolo_to_movebase/camera_config.hpp"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <vision_msgs/Detection2DArray.h>
#include <std_msgs/String.h>          // 运行时模式切换用

#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <string>
#include <vector>
#include <cstdint>   // int64_t

namespace yolo2mb {

class YoloToMoveBase {
 public:
  YoloToMoveBase(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    loadParams();
    if (!valid_) return;
    setupRos();
    printSummary();
  }

  bool valid() const { return valid_; }

 private:
  // ================= 参数加载 =================
  void loadParams() {
    pnh_.param<std::string>("world_frame", world_frame_, "world");
    pnh_.param<bool>("use_full_attitude", use_full_attitude_, true);
    double delay = 8.0;
    pnh_.param<double>("publish_delay", delay, delay);
    publish_delay_ = ros::Duration(delay);
    pnh_.param<double>("tf_rate", tf_rate_, 10.0);

    std::map<std::string, std::string> cn;
    if (pnh_.getParam("class_names", cn)) {
      for (const auto& kv : cn) {
        try { class_names_[std::stoi(kv.first)] = kv.second; }
        catch (const std::exception&) { /* key 不是数字则忽略 */ }
      }
    }

    std::vector<std::string> cam_names;
    if (!pnh_.getParam("cameras", cam_names) || cam_names.empty()) {
      ROS_FATAL("缺少参数 cameras, 请检查 cameras.yaml 是否被正确加载");
      ros::shutdown();
      return;
    }
    for (const auto& n : cam_names)
      cameras_.emplace(n, CameraConfig::fromParams(pnh_, n));

    valid_ = true;
  }

  void setupRos() {
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    // odom 话题可由 launch/yaml 覆盖, 默认值填原来写死的那个
    std::string odom_topic;
    pnh_.param<std::string>("odom_topic", odom_topic, std::string("/mavros/local_position/odom"));
    odom_sub_ = nh_.subscribe(odom_topic, 1, &YoloToMoveBase::odomCallback, this);

    // 运行时测距模式切换: 发布 "相机名:模式" (ground/distance/off), 如 "down:ground"
    mode_sub_ = nh_.subscribe("/yolo_to_movebase/set_depth_mode", 1,
                              &YoloToMoveBase::modeSwitchCallback, this);

    // 每个相机一个订阅者, 用 std::bind 绑定相机名
    for (const auto& kv : cameras_) {
      subs_.push_back(nh_.subscribe<vision_msgs::Detection2DArray>(
          kv.second.topic, 1,
          std::bind(&YoloToMoveBase::detectionCallback, this,
                    std::placeholders::_1, kv.first)));
    }
    tf_timer_ = nh_.createTimer(ros::Duration(1.0 / tf_rate_),
                                [this](const ros::TimerEvent&) { broadcastTf(); });
  }

  void printSummary() const {
    ROS_INFO("YoloToMoveBase 启动 (C++/Noetic)  frame=%s  full_attitude=%s  delay=%.1fs",
             world_frame_.c_str(), use_full_attitude_ ? "true" : "false",
             publish_delay_.toSec());
    for (const auto& kv : cameras_) {
      const auto& c = kv.second;
      const std::string tilt_str = c.max_tilt_deg > 0.0
          ? std::to_string(c.max_tilt_deg) : std::string("none");
      ROS_INFO("  [%s] topic=%s  mode=%s  tilt_limit=%s  offset=(%.2f, %.2f)",
               kv.first.c_str(), c.topic.c_str(), c.depth_mode.c_str(),
               tilt_str.c_str(), c.x_offset, c.z_offset);
    }
  }

  // ================= 里程计回调 =================
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    const auto& p = msg->pose.pose.position;
    const auto& o = msg->pose.pose.orientation;
    drone_p_ = Eigen::Vector3d(p.x, p.y, p.z);
    drone_q_ = Eigen::Quaterniond(o.w, o.x, o.y, o.z);
    drone_yaw_ = std::atan2(
        2.0 * (drone_q_.w() * drone_q_.z() + drone_q_.x() * drone_q_.y()),
        1.0 - 2.0 * (drone_q_.y() * drone_q_.y() + drone_q_.z() * drone_q_.z()));

    // 完整姿态 or 仅yaw (水平悬停时两者等价, 保证与旧版结果一致)
    R_wb_ = use_full_attitude_
                ? drone_q_.toRotationMatrix()
                : Eigen::AngleAxisd(drone_yaw_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T_wb_ = drone_p_;
    if (!has_odom_) ROS_INFO("收到第一帧里程计位姿");
    has_odom_ = true;
  }

  // ================= 运行时模式切换 =================
  void modeSwitchCallback(const std_msgs::String::ConstPtr& msg) {
    // 格式: "相机名:模式", 如 "down:ground" / "front:distance" / "down:off"
    const auto pos = msg->data.find(':');
    if (pos == std::string::npos) {
      ROS_WARN("模式切换格式错误: '%s' (应为 相机名:ground|distance|off)",
               msg->data.c_str());
      return;
    }
    const std::string name = msg->data.substr(0, pos);
    const std::string mode = msg->data.substr(pos + 1);
    if (mode != "ground" && mode != "distance" && mode != "off") {
      ROS_WARN("未知模式 '%s', 仅支持 ground / distance / off", mode.c_str());
      return;
    }
    const auto it = cameras_.find(name);
    if (it == cameras_.end()) {
      std::string names;
      for (const auto& kv : cameras_) names += kv.first + " ";
      ROS_WARN("相机 '%s' 不存在, 可选: %s", name.c_str(), names.c_str());
      return;
    }
    it->second.depth_mode = mode;
    ROS_INFO("[%s] 测距模式已切换为 %s", name.c_str(), mode.c_str());
  }

  // ================= 检测回调 =================
  void detectionCallback(const vision_msgs::Detection2DArray::ConstPtr& msg,
                         const std::string& cam_name) {
    const auto it = cameras_.find(cam_name);
    if (it == cameras_.end()) return;
    const CameraConfig& cam = it->second;

    if (cam.depth_mode == "off") return;   // 运行时已关闭该相机

    if (!has_odom_) {
      ROS_WARN_THROTTLE(5.0, "[%s] 尚未收到里程计, 丢弃检测结果", cam_name.c_str());
      return;
    }

    // 1. 选置信度最高且达标的检测
    const vision_msgs::Detection2D* best = nullptr;
    double best_score = 0.0;
    for (const auto& det : msg->detections) {
      if (det.results.empty()) continue;
      const double s = det.results[0].score;
      if (s >= cam.min_score && s > best_score) { best = &det; best_score = s; }
    }
    if (best == nullptr) return;

    // 2. 提取像素中心 / 框高 / 类别
    const double u = best->bbox.center.x;
    const double v = best->bbox.center.y;
    const double box_h = best->bbox.size_y;
    const std::string cls = resolveClassName(best->results[0].id);
    ROS_INFO("[%s] 检测到 %s  score=%.2f  px=(%.0f, %.0f)",
             cam_name.c_str(), cls.c_str(), best_score, u, v);

    // 3. 倾斜保护: 倾斜大时像素误差会被放大成地面位置误差
    if (cam.max_tilt_deg > 0.0 && tiltDeg() > cam.max_tilt_deg) {
      ROS_WARN_THROTTLE(2.0, "[%s] 机体倾斜 %.1f° > %.1f°, 丢弃检测",
                        cam_name.c_str(), tiltDeg(), cam.max_tilt_deg);
      return;
    }

    // 4. 像素 -> 世界坐标
    Eigen::Vector3d P = Eigen::Vector3d::Zero();
    double dist = 0.0;
    const bool ok = (cam.depth_mode == "ground")
        ? pixelToWorldGround(cam, u, v, P, dist)
        : pixelToWorldDistance(cam, u, v, box_h, cls, P, dist);
    if (!ok) return;
    ROS_INFO("[%s] 目标世界坐标: (%.2f, %.2f, %.2f)  距离: %.2f m",
             cam_name.c_str(), P.x(), P.y(), P.z(), dist);

    // 5. 更新状态
    target_world_ = P;
    current_cam_ = cam_name;
    last_class_ = cls;
    targets_[cam_name + "/" + cls] = P;

    // 6. 节流发布(与Python版一致: 最小间隔8s)
    const ros::Time now = ros::Time::now();
    const ros::Duration elapsed = now - last_publish_time_;
    if (elapsed >= publish_delay_) {
      publishGoal(cam, cls);
      last_publish_time_ = now;
    } else {
      const ros::Duration remain = publish_delay_ - elapsed;
      ROS_INFO("[%s] 距上次发布仅 %.1fs, %.1fs 后发布目标点",
               cam_name.c_str(), elapsed.toSec(), remain.toSec());
      publish_timer_.stop();   // 取消上一次的待发布定时器
      publish_timer_ = nh_.createTimer(
          remain, std::bind(&YoloToMoveBase::delayedPublish, this, cam_name, cls),
          true /* oneshot */);
    }
  }

  // ================= 两种测距/变换模式 =================
  /// ground模式: 像素射线与地平面(z=ground_z)求交, 深度来自里程计高度
  bool pixelToWorldGround(const CameraConfig& cam, double u, double v,
                          Eigen::Vector3d& P_world, double& dist) const {
    const Eigen::Vector3d d_cam((u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, 1.0);
    const Eigen::Vector3d d_world = R_wb_ * (cam.R_cb * d_cam);
    const Eigen::Vector3d C_world = T_wb_ + R_wb_ * cam.mount_xyz;

    if (std::abs(d_world.z()) < 1e-6) {
      ROS_WARN_THROTTLE(2.0, "[%s] 射线与地面平行, 无法求交", cam.name.c_str());
      return false;
    }
    const double t = (cam.ground_z - C_world.z()) / d_world.z();
    if (t <= 0.0) {
      ROS_WARN_THROTTLE(2.0, "[%s] 交点在相机背后(t=%.2f), 数据无效", cam.name.c_str(), t);
      return false;
    }
    dist = t * d_cam.norm();
    if (dist < cam.min_range || dist > cam.max_range) {
      ROS_WARN_THROTTLE(2.0, "[%s] 距离 %.2fm 超出 [%.1f, %.1f]",
                        cam.name.c_str(), dist, cam.min_range, cam.max_range);
      return false;
    }
    P_world = C_world + t * d_world;
    return true;
  }

  /// distance模式: 相似三角形 Z = fx*H/h_px, 再反投影到世界系
  bool pixelToWorldDistance(const CameraConfig& cam, double u, double v,
                            double box_h_px, const std::string& cls,
                            Eigen::Vector3d& P_world, double& dist) const {
    const auto sz = cam.real_sizes.find(cls);
    if (sz == cam.real_sizes.end() || box_h_px <= 1.0) {
      ROS_WARN_THROTTLE(2.0, "[%s] 类别'%s'未配置real_sizes或框过小, 无法测距",
                        cam.name.c_str(), cls.c_str());
      return false;
    }
    const double Z = cam.fx * sz->second / box_h_px;
    if (Z < cam.min_range || Z > cam.max_range) {
      ROS_WARN_THROTTLE(2.0, "[%s] 距离 %.2fm 超出 [%.1f, %.1f]",
                        cam.name.c_str(), Z, cam.min_range, cam.max_range);
      return false;
    }
    const Eigen::Vector3d p_cam((u - cam.cx) / cam.fx * Z,
                                (v - cam.cy) / cam.fy * Z, Z);
    const Eigen::Vector3d p_body = cam.mount_xyz + cam.R_cb * p_cam;
    P_world = T_wb_ + R_wb_ * p_body;
    dist = Z;
    return true;
  }

  // ================= 输出 =================
  void publishGoal(const CameraConfig& cam, const std::string& cls) {
    const Eigen::Vector3d goal_p =
        target_world_ + Eigen::Vector3d(cam.x_offset, 0.0, cam.z_offset);

    geometry_msgs::PoseStamped goal;
    goal.header.seq = seq_++;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = world_frame_;
    goal.pose.position.x = goal_p.x();
    goal.pose.position.y = goal_p.y();
    goal.pose.position.z = goal_p.z();

    // 航向保持当前yaw
    const Eigen::Quaterniond q_yaw(
        Eigen::AngleAxisd(drone_yaw_, Eigen::Vector3d::UnitZ()));
    goal.pose.orientation.x = q_yaw.x();
    goal.pose.orientation.y = q_yaw.y();
    goal.pose.orientation.z = q_yaw.z();
    goal.pose.orientation.w = q_yaw.w();

    goal_pub_.publish(goal);
    ROS_INFO("已发布目标 '%s'(%s相机) -> (%.2f, %.2f, %.2f)  偏移(x:+%.2f, z:+%.2f)",
             cls.c_str(), cam.name.c_str(),
             goal_p.x(), goal_p.y(), goal_p.z(), cam.x_offset, cam.z_offset);
  }

  void delayedPublish(const std::string& cam_name, const std::string& cls) {
    const auto it = cameras_.find(cam_name);
    if (it == cameras_.end()) return;
    publishGoal(it->second, cls);
    last_publish_time_ = ros::Time::now();
  }

  // ================= TF广播 =================
  void broadcastTf() {
    if (!valid_) return;

    if (has_odom_) {
      // 机体: world -> drone_current
      geometry_msgs::TransformStamped t;
      fillTf(t, world_frame_, drone_p_, drone_q_, "drone_current");
      tf_bc_.sendTransform(t);

      // 相机: drone_current -> cam_x (挂在机体下, 跟着飞机飞)  ← 修复点
      for (const auto& kv : cameras_) {
        geometry_msgs::TransformStamped tc;
        fillTf(tc, "drone_current", kv.second.mount_xyz,
               Eigen::Quaterniond(kv.second.R_cb), "cam_" + kv.first);
        tf_bc_.sendTransform(tc);
      }
    }

    if (!targets_.empty()) {
      const auto it = cameras_.find(current_cam_);
      const double xo = (it != cameras_.end()) ? it->second.x_offset : 0.0;
      const double zo = (it != cameras_.end()) ? it->second.z_offset : 0.0;

      // 目标是世界系坐标, 挂在 world 下
      geometry_msgs::TransformStamped t1;
      fillTf(t1, world_frame_, target_world_, Eigen::Quaterniond::Identity(),
             "target_original_" + current_cam_);
      tf_bc_.sendTransform(t1);

      geometry_msgs::TransformStamped t2;
      fillTf(t2, world_frame_, target_world_ + Eigen::Vector3d(xo, 0.0, zo),
             Eigen::Quaterniond::Identity(), "target_final_" + current_cam_);
      tf_bc_.sendTransform(t2);
    }

    int i = 0;
    for (const auto& kv : targets_) {
      std::string child = "target_" + std::to_string(i++) + "_" + kv.first;
      std::replace(child.begin(), child.end(), '/', '_');  // TF帧名不能含'/'
      geometry_msgs::TransformStamped t;
      fillTf(t, world_frame_, kv.second, Eigen::Quaterniond::Identity(), child);
      tf_bc_.sendTransform(t);
    }
  }

  void fillTf(geometry_msgs::TransformStamped& t, const std::string& parent,
              const Eigen::Vector3d& p, const Eigen::Quaterniond& q,
              const std::string& child) const {
    t.header.stamp = ros::Time::now();
    t.header.frame_id = parent;   // 父帧由调用方指定, 不再写死 world
    t.child_frame_id = child;
    t.transform.translation.x = p.x();
    t.transform.translation.y = p.y();
    t.transform.translation.z = p.z();
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
  }

  // ================= 工具 =================
  /// id 为 int64 的 vision_msgs 版本（apt 安装的 ros-noetic-vision-msgs 就是这种）
  std::string resolveClassName(std::int64_t raw_id) const {
    const auto it = class_names_.find(static_cast<int>(raw_id));
    return it != class_names_.end() ? it->second : ("class_" + std::to_string(raw_id));
  }
  /// id 为 string 的 vision_msgs 版本（源码安装的新版可能是这种，留作兼容）
  std::string resolveClassName(const std::string& raw_id) const {
    try {
      const int idx = std::stoi(raw_id);
      const auto it = class_names_.find(idx);
      return it != class_names_.end() ? it->second : ("class_" + std::to_string(idx));
    } catch (const std::exception&) {
      return raw_id;
    }
  }

  double tiltDeg() const {
    const double c = 2.0 * (drone_q_.w() * drone_q_.w() + drone_q_.z() * drone_q_.z()) - 1.0;
    return std::acos(std::max(-1.0, std::min(1.0, c))) * 180.0 / M_PI;
  }

  // ================= 成员 =================
  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;
  bool valid_ = false;

  std::string world_frame_ = "world";
  bool use_full_attitude_ = true;
  double tf_rate_ = 10.0;
  ros::Duration publish_delay_{8.0};

  std::map<int, std::string> class_names_;
  std::map<std::string, CameraConfig> cameras_;

  bool has_odom_ = false;
  Eigen::Vector3d drone_p_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond drone_q_ = Eigen::Quaterniond::Identity();
  double drone_yaw_ = 0.0;
  Eigen::Matrix3d R_wb_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d T_wb_ = Eigen::Vector3d::Zero();

  Eigen::Vector3d target_world_ = Eigen::Vector3d::Zero();
  std::string current_cam_;
  std::string last_class_;
  std::map<std::string, Eigen::Vector3d> targets_;
  int seq_ = 1;

  ros::Publisher goal_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber mode_sub_;
  std::vector<ros::Subscriber> subs_;
  ros::Timer tf_timer_;
  ros::Timer publish_timer_;
  tf2_ros::TransformBroadcaster tf_bc_;

  ros::Time last_publish_time_{0};
};

}  // namespace yolo2mb

int main(int argc, char** argv) {
  ros::init(argc, argv, "yolo_to_movebase");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  yolo2mb::YoloToMoveBase node(nh, pnh);
  if (node.valid()) ros::spin();
  return 0;
}
