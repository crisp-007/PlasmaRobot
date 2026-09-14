#include <rclcpp/rclcpp.hpp>
#include <realsense2_camera_msgs/msg/extrinsics.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace
{

constexpr std::uint8_t kFallbackColor = 96;

std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time &stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

bool findFloatField(const sensor_msgs::msg::PointCloud2 &cloud,
                    const std::string &name,
                    std::uint32_t &offset)
{
  for (const auto &field : cloud.fields) {
    if (field.name == name &&
        field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
        field.count == 1) {
      offset = field.offset;
      return true;
    }
  }
  return false;
}

sensor_msgs::msg::PointField makeFloatField(const std::string &name,
                                             std::uint32_t offset)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  return field;
}

}  // namespace

class ColoredPointCloudNode : public rclcpp::Node
{
public:
  ColoredPointCloudNode()
  : Node("plasma_colored_pointcloud"),
    last_process_(std::chrono::steady_clock::time_point::min())
  {
    input_cloud_topic_ = declare_parameter<std::string>(
      "input_cloud_topic", "/camera/depth/color/points");
    color_topic_ = declare_parameter<std::string>(
      "color_topic", "/camera/color/image_raw");
    color_info_topic_ = declare_parameter<std::string>(
      "color_info_topic", "/camera/color/camera_info");
    extrinsics_topic_ = declare_parameter<std::string>(
      "extrinsics_topic", "/camera/extrinsics/depth_to_color");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/plasma/camera/colored_points");
    max_output_hz_ = std::max(
      0.1, declare_parameter<double>("max_output_hz", 6.0));
    max_color_age_sec_ = std::max(
      0.0, declare_parameter<double>("max_color_age_sec", 1.0));

    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    const auto extrinsics_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable().transient_local();

    output_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    color_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      color_topic_, reliable_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        latest_color_ = std::move(message);
      });
    color_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      color_info_topic_, reliable_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        color_info_ = std::move(message);
      });
    extrinsics_subscription_ =
      create_subscription<realsense2_camera_msgs::msg::Extrinsics>(
        extrinsics_topic_, extrinsics_qos,
        [this](realsense2_camera_msgs::msg::Extrinsics::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(cache_mutex_);
          depth_to_color_ = std::move(message);
        });
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ColoredPointCloudNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Color bridge ready: %s -> %s at up to %.1f Hz",
      input_cloud_topic_.c_str(), output_topic_.c_str(), max_output_hz_);
  }

private:
  struct ProjectionCache
  {
    sensor_msgs::msg::Image::ConstSharedPtr color;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
    realsense2_camera_msgs::msg::Extrinsics::ConstSharedPtr extrinsics;
  };

  ProjectionCache projectionCache() const
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return {latest_color_, color_info_, depth_to_color_};
  }

  bool colorIsUsable(const sensor_msgs::msg::PointCloud2 &cloud,
                     const ProjectionCache &cache) const
  {
    if (!cache.color || !cache.info || !cache.extrinsics ||
        cache.info->k[0] <= 0.0 || cache.info->k[4] <= 0.0 ||
        cache.color->width == 0 || cache.color->height == 0) {
      return false;
    }
    const std::int64_t age_ns = std::llabs(
      stampNanoseconds(cloud.header.stamp) -
      stampNanoseconds(cache.color->header.stamp));
    return static_cast<double>(age_ns) * 1e-9 <= max_color_age_sec_;
  }

  bool sampleColor(const sensor_msgs::msg::Image &image,
                   int u, int v,
                   std::uint8_t &red,
                   std::uint8_t &green,
                   std::uint8_t &blue) const
  {
    if (u < 0 || v < 0 || u >= static_cast<int>(image.width) ||
        v >= static_cast<int>(image.height)) {
      return false;
    }

    int channels = 0;
    bool rgb_order = true;
    if (image.encoding == sensor_msgs::image_encodings::RGB8) {
      channels = 3;
    } else if (image.encoding == sensor_msgs::image_encodings::BGR8) {
      channels = 3;
      rgb_order = false;
    } else if (image.encoding == sensor_msgs::image_encodings::RGBA8) {
      channels = 4;
    } else if (image.encoding == sensor_msgs::image_encodings::BGRA8) {
      channels = 4;
      rgb_order = false;
    } else {
      return false;
    }

    const std::size_t offset = static_cast<std::size_t>(v) * image.step +
      static_cast<std::size_t>(u) * channels;
    if (offset + static_cast<std::size_t>(channels) > image.data.size()) {
      return false;
    }
    if (rgb_order) {
      red = image.data[offset];
      green = image.data[offset + 1];
      blue = image.data[offset + 2];
    } else {
      blue = image.data[offset];
      green = image.data[offset + 1];
      red = image.data[offset + 2];
    }
    return true;
  }

  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    const auto now = std::chrono::steady_clock::now();
    const auto minimum_interval = std::chrono::duration<double>(1.0 / max_output_hz_);
    if (last_process_ != std::chrono::steady_clock::time_point::min() &&
        now - last_process_ < minimum_interval) {
      return;
    }
    last_process_ = now;

    if (!cloud || cloud->is_bigendian || cloud->point_step == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring unsupported empty or big-endian pointcloud");
      return;
    }

    std::uint32_t x_offset = 0;
    std::uint32_t y_offset = 0;
    std::uint32_t z_offset = 0;
    if (!findFloatField(*cloud, "x", x_offset) ||
        !findFloatField(*cloud, "y", y_offset) ||
        !findFloatField(*cloud, "z", z_offset)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input pointcloud does not contain float32 x/y/z fields");
      return;
    }

    const std::size_t input_count =
      static_cast<std::size_t>(cloud->width) * cloud->height;
    std::vector<cv::Point3f> points;
    points.reserve(input_count);
    for (std::size_t index = 0; index < input_count; ++index) {
      const std::size_t base = index * cloud->point_step;
      if (base + std::max({x_offset, y_offset, z_offset}) + sizeof(float) >
          cloud->data.size()) {
        break;
      }
      float x = 0.0F;
      float y = 0.0F;
      float z = 0.0F;
      std::memcpy(&x, cloud->data.data() + base + x_offset, sizeof(float));
      std::memcpy(&y, cloud->data.data() + base + y_offset, sizeof(float));
      std::memcpy(&z, cloud->data.data() + base + z_offset, sizeof(float));
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && z > 0.0F) {
        points.emplace_back(x, y, z);
      }
    }
    if (points.empty()) {
      return;
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header = cloud->header;
    output.height = 1;
    output.width = static_cast<std::uint32_t>(points.size());
    output.fields = {
      makeFloatField("x", 0), makeFloatField("y", 4),
      makeFloatField("z", 8), makeFloatField("rgb", 12)};
    output.is_bigendian = false;
    output.point_step = 16;
    output.row_step = output.point_step * output.width;
    output.is_dense = true;
    output.data.resize(output.row_step);

    for (std::size_t index = 0; index < points.size(); ++index) {
      auto *destination = output.data.data() + index * output.point_step;
      std::memcpy(destination, &points[index].x, sizeof(float));
      std::memcpy(destination + 4, &points[index].y, sizeof(float));
      std::memcpy(destination + 8, &points[index].z, sizeof(float));
      const std::uint32_t fallback =
        (static_cast<std::uint32_t>(kFallbackColor) << 16) |
        (static_cast<std::uint32_t>(kFallbackColor) << 8) |
        kFallbackColor;
      std::memcpy(destination + 12, &fallback, sizeof(fallback));
    }

    const ProjectionCache cache = projectionCache();
    std::size_t colored_points = 0;
    if (colorIsUsable(*cloud, cache)) {
      cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        cache.info->k[0], cache.info->k[1], cache.info->k[2],
        cache.info->k[3], cache.info->k[4], cache.info->k[5],
        cache.info->k[6], cache.info->k[7], cache.info->k[8]);
      cv::Mat distortion(1, static_cast<int>(cache.info->d.size()), CV_64F);
      for (std::size_t i = 0; i < cache.info->d.size(); ++i) {
        distortion.at<double>(0, static_cast<int>(i)) = cache.info->d[i];
      }

      // librealsense publishes rs2_extrinsics.rotation in column-major order.
      cv::Mat rotation(3, 3, CV_64F);
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          rotation.at<double>(row, column) =
            cache.extrinsics->rotation[column * 3 + row];
        }
      }
      cv::Mat rotation_vector;
      cv::Rodrigues(rotation, rotation_vector);
      cv::Mat translation = (cv::Mat_<double>(3, 1) <<
        cache.extrinsics->translation[0],
        cache.extrinsics->translation[1],
        cache.extrinsics->translation[2]);

      std::vector<cv::Point2f> pixels;
      cv::projectPoints(
        points, rotation_vector, translation, camera_matrix, distortion, pixels);
      for (std::size_t index = 0; index < pixels.size(); ++index) {
        const int u = static_cast<int>(std::lround(pixels[index].x));
        const int v = static_cast<int>(std::lround(pixels[index].y));
        std::uint8_t red = 0;
        std::uint8_t green = 0;
        std::uint8_t blue = 0;
        if (!sampleColor(*cache.color, u, v, red, green, blue)) {
          continue;
        }
        const std::uint32_t packed =
          (static_cast<std::uint32_t>(red) << 16) |
          (static_cast<std::uint32_t>(green) << 8) |
          blue;
        std::memcpy(
          output.data.data() + index * output.point_step + 12,
          &packed, sizeof(packed));
        ++colored_points;
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "RGB image/calibration is unavailable or stale; publishing gray geometry");
    }

    output_publisher_->publish(output);
    ++published_frames_;
    if (published_frames_ == 1 || published_frames_ % 60 == 0) {
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - now).count();
      RCLCPP_INFO(
        get_logger(), "Published %zu points (%zu colored) in %.1f ms",
        points.size(), colored_points, elapsed_ms);
    }
  }

  std::string input_cloud_topic_;
  std::string color_topic_;
  std::string color_info_topic_;
  std::string extrinsics_topic_;
  std::string output_topic_;
  double max_output_hz_ = 6.0;
  double max_color_age_sec_ = 1.0;
  std::chrono::steady_clock::time_point last_process_;
  std::size_t published_frames_ = 0;

  mutable std::mutex cache_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_color_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr color_info_;
  realsense2_camera_msgs::msg::Extrinsics::ConstSharedPtr depth_to_color_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
    color_info_subscription_;
  rclcpp::Subscription<realsense2_camera_msgs::msg::Extrinsics>::SharedPtr
    extrinsics_subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColoredPointCloudNode>());
  rclcpp::shutdown();
  return 0;
}
