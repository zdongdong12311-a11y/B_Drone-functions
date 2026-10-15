#pragma once

#include <cstdint>   // int64_t

#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <ros/ros.h>

namespace yolo2mb {

/// 相机光学系(x右,y下,z前) -> 机体系FLU(x前,y左,z上)
/// 列1: 图像右->机体右(-Y)  列2: 图像下->机体下(-Z)  列3: 光轴->机体前(+X)
inline Eigen::Matrix3d opticalToFlu() {
  Eigen::Matrix3d R;
  R <<  0.0, 0.0, 1.0,
       -1.0, 0.0, 0.0,
        0.0,-1.0, 0.0;
  return R;
}

/// 欧拉角(度) -> 旋转矩阵, R = Rz(yaw)*Ry(pitch)*Rx(roll)
/// 修正: AngleAxis 链式相乘结果是四元数, 必须显式 .toRotationMatrix()
inline Eigen::Matrix3d rpyDegToMatrix(double roll_deg, double pitch_deg, double yaw_deg) {
  const double r = roll_deg  * M_PI / 180.0;
  const double p = pitch_deg * M_PI / 180.0;
  const double y = yaw_deg   * M_PI / 180.0;
  return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

/// 单个相机的全部配置
struct CameraConfig {
  std::string name;
  std::string topic;
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;

  Eigen::Vector3d mount_xyz = Eigen::Vector3d::Zero();  ///< 相机在机体系中的安装位置
  Eigen::Matrix3d R_cb;                                 ///< 光学系 -> 机体系

  std::string depth_mode = "ground";                    ///< "ground" | "distance"
  double ground_z = 0.0;                                ///< ground模式: 地面世界高度
  std::map<std::string, double> real_sizes;             ///< distance模式: 类别->真实高度

  double min_range = 0.5, max_range = 80.0;
  double min_score = 0.5;
  double max_tilt_deg = -1.0;                           ///< <=0 表示不限制
  double x_offset = 0.0, z_offset = 0.0;                ///< 目标点固定偏移

  static CameraConfig fromParams(const ros::NodeHandle& pnh, const std::string& cam_name);
};

}  // namespace yolo2mb