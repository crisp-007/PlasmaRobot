#ifndef PLASMA_PATH_TRANSFORM__PATH_TRANSFORMER_HPP_
#define PLASMA_PATH_TRANSFORM__PATH_TRANSFORMER_HPP_

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <string>

namespace plasma_path_transform
{

struct TransformCalibration
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  bool loaded = false;
  bool validated = false;
  std::string id;
  std::string frame_id;
  std::string source;
};

class PathTransformer
{
public:
  static TransformCalibration loadTransformYaml(
    const std::string & path, const std::string & matrix_key);
  static Eigen::Isometry3d loadRigidTransformYaml(
    const std::string & path, const std::string & matrix_key);

  static bool validateRigidTransform(
    const Eigen::Matrix4d & matrix, std::string * reason = nullptr);

  static Eigen::Isometry3d poseToIsometry(
    const geometry_msgs::msg::Pose & pose);
  static geometry_msgs::msg::Pose isometryToPose(
    const Eigen::Isometry3d & transform);
  static geometry_msgs::msg::Point transformPoint(
    const Eigen::Isometry3d & transform,
    const geometry_msgs::msg::Point & point);
  static Eigen::Isometry3d translatedAlongAxis(
    const Eigen::Isometry3d & transform,
    const Eigen::Vector3d & axis,
    double distance_m);
  static Eigen::Isometry3d tcpPoseForNozzleTipPose(
    const Eigen::Isometry3d & nozzle_tip_pose,
    const Eigen::Isometry3d & tcp_to_nozzle_tip);
};

}  // namespace plasma_path_transform

#endif  // PLASMA_PATH_TRANSFORM__PATH_TRANSFORMER_HPP_
