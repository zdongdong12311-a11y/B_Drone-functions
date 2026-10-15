#include "yolo_to_movebase/camera_config.hpp"

namespace yolo2mb {

CameraConfig CameraConfig::fromParams(const ros::NodeHandle& pnh, const std::string& cam_name) {
  CameraConfig c;
  c.name = cam_name;
  ros::NodeHandle nh(pnh, cam_name);   // 私有命名空间下的 <cam_name> 节点

  nh.param<std::string>("topic", c.topic, "/yolo_msgs");
  nh.param<double>("fx", c.fx, 1000.0);
  nh.param<double>("fy", c.fy, c.fx);
  nh.param<double>("cx", c.cx, 640.0);
  nh.param<double>("cy", c.cy, 360.0);

  std::vector<double> rpy_def{0.0, 0.0, 0.0};
  std::vector<double> xyz_def{0.0, 0.0, 0.0};
  std::vector<double> rpy_deg, xyz;
  nh.param<std::vector<double>>("mount_rpy_deg", rpy_deg, rpy_def);
  nh.param<std::vector<double>>("mount_xyz", xyz, xyz_def);
  if (rpy_deg.size() != 3 || xyz.size() != 3) {
    ROS_ERROR("[%s] mount_rpy_deg / mount_xyz 必须是3个元素", cam_name.c_str());
  }
  c.mount_xyz = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
  c.R_cb = rpyDegToMatrix(rpy_deg[0], rpy_deg[1], rpy_deg[2]) * opticalToFlu();

  nh.param<std::string>("depth_mode", c.depth_mode, std::string("ground"));
  nh.param<double>("ground_z", c.ground_z, 0.0);
  nh.param<double>("min_range", c.min_range, 0.5);
  nh.param<double>("max_range", c.max_range, 80.0);
  nh.param<double>("min_score", c.min_score, 0.5);
  nh.param<double>("max_tilt_deg", c.max_tilt_deg, -1.0);
  nh.param<double>("x_offset", c.x_offset, 0.0);
  nh.param<double>("z_offset", c.z_offset, 0.0);

  std::map<std::string, double> sizes;
  if (nh.getParam("real_sizes", sizes)) c.real_sizes = sizes;

  return c;
}

}  // namespace yolo2mb