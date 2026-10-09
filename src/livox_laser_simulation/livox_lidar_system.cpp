#include "livox_laser_simulation/livox_lidar_system.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <utility>

namespace livox_laser_simulation
{
namespace
{
template<typename T>
T SdfValue(const std::shared_ptr<const sdf::Element> &_sdf,
           const std::string &_name, const T &_default)
{
  return _sdf->HasElement(_name) ? _sdf->Get<T>(_name) : _default;
}

void AddField(sensor_msgs::msg::PointCloud2 &_msg, const std::string &_name,
              uint32_t _offset, uint8_t _datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = _name;
  field.offset = _offset;
  field.datatype = _datatype;
  field.count = 1;
  _msg.fields.push_back(field);
}

template<typename T>
void WriteValue(std::vector<uint8_t> &_data, size_t _offset, const T &_value)
{
  std::memcpy(_data.data() + _offset, &_value, sizeof(T));
}

}  // namespace

void LivoxLidarSystem::Configure(const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_event_mgr)
{
  (void)_event_mgr;
  const auto sensor_component = _ecm.Component<gz::sim::components::Sensor>(_entity);
  const auto name_component = _ecm.Component<gz::sim::components::Name>(_entity);
  const auto parent_component =
    _ecm.Component<gz::sim::components::ParentEntity>(_entity);
  if (!sensor_component || !name_component || !parent_component)
  {
    gzerr << "LivoxLidarSystem must be attached to a sensor entity.\n";
    return;
  }

  sensor_name_ = name_component->Data();

  csv_file_name_ = SdfValue<std::string>(_sdf, "csv_file_name", "");
  gpu_topic_ = SdfValue<std::string>(_sdf, "gpu_topic", gpu_topic_);
  ros_topic_ = SdfValue<std::string>(_sdf, "ros_topic", ros_topic_);
  frame_name_ = SdfValue<std::string>(_sdf, "frame_name", sensor_name_);
  sample_step_ = std::max(1, SdfValue<int>(_sdf, "samples", sample_step_));
  downsample_ = std::max(1, SdfValue<int>(_sdf, "downsample", downsample_));
  line_count_ = SdfValue<int>(_sdf, "line_count", line_count_);
  startup_skip_scans_ = static_cast<size_t>(std::max(
    0, SdfValue<int>(_sdf, "startup_skip_scans", 0)));
  min_dist_ = SdfValue<double>(_sdf, "min_range", min_dist_);
  max_dist_ = SdfValue<double>(_sdf, "max_range", max_dist_);
  point_rate_ = SdfValue<double>(_sdf, "point_rate", point_rate_);
  use_inf_ = SdfValue<bool>(_sdf, "use_inf", use_inf_);
  publish_point_time_offsets_ = SdfValue<bool>(
    _sdf, "publish_point_time_offsets", publish_point_time_offsets_);

  if (line_count_ < 1 || line_count_ > 256 || point_rate_ <= 0.0)
  {
    gzerr << "line_count must be in [1, 256] and point_rate must be positive.\n";
    return;
  }

  const int output_type = SdfValue<int>(_sdf, "publish_pointcloud_type", 2);
  if (output_type < 1 || output_type > 4)
  {
    gzerr << "Invalid publish_pointcloud_type " << output_type << ".\n";
    return;
  }
  publish_pointcloud_type_ = static_cast<PointCloudType>(output_type);

  if (!csv_file_name_.empty() && !std::filesystem::path(csv_file_name_).is_absolute())
  {
    csv_file_name_ = (std::filesystem::path(
      ament_index_cpp::get_package_share_directory("vl400_sim")) /
      "scan_patterns" / csv_file_name_).string();
  }
  if (csv_file_name_.empty() || !std::filesystem::exists(csv_file_name_))
  {
    gzerr << "Scan pattern does not exist: [" << csv_file_name_ << "].\n";
    return;
  }

  depth_sampler_ = std::make_unique<LivoxDepthSampler>();
  depth_sampler_->SetParameters(downsample_, min_dist_, max_dist_, use_inf_);
  if (!depth_sampler_->LoadScanPattern(
      csv_file_name_, line_count_, point_rate_))
  {
    gzerr << "Failed to load scan pattern: [" << csv_file_name_ << "].\n";
    return;
  }

  InitializeROS();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    configured_ = true;
  }
  if (!transport_node_.Subscribe(
      gpu_topic_, &LivoxLidarSystem::OnGpuScan, this))
  {
    gzerr << "Failed to subscribe to GPU lidar topic [" << gpu_topic_
           << "].\n";
    std::lock_guard<std::mutex> lock(mutex_);
    configured_ = false;
    return;
  }

  gzmsg << "Livox lidar [" << sensor_name_ << "] publishing [" << ros_topic_
        << "] with " << depth_sampler_->PointCount()
        << " scan pattern points from GPU topic [" << gpu_topic_ << "].\n";
}

void LivoxLidarSystem::InitializeROS()
{
  if (!rclcpp::ok())
  {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
  ros_node_ = std::make_shared<rclcpp::Node>("livox_lidar_sim_" + sensor_name_);
  if (publish_pointcloud_type_ ==
      PointCloudType::LIVOX_LASER_SIMULATION_CUSTOM_MSG)
  {
    custom_msg_pub_ = ros_node_->create_publisher<msg::CustomMsg>(ros_topic_, 5);
  }
  else
  {
    pointcloud_pub_ =
      ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(ros_topic_, 5);
  }
}

void LivoxLidarSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
                                  const gz::sim::EntityComponentManager &_ecm)
{
  if (!configured_ || _info.paused)
    return;

  std::lock_guard<std::mutex> lock(mutex_);
  sim_time_ = _info.simTime;
}

void LivoxLidarSystem::OnGpuScan(const gz::msgs::LaserScan &_scan)
{
  const unsigned int width = _scan.count();
  const unsigned int height = std::max(1u, _scan.vertical_count());
  const size_t pixel_count = static_cast<size_t>(width) * height;
  if (width < 2 || height < 2 ||
      static_cast<size_t>(_scan.ranges_size()) < pixel_count)
  {
    gzerr << "Invalid GPU distance image on [" << gpu_topic_ << "]: "
           << width << "x" << height << " with " << _scan.ranges_size()
           << " ranges.\n";
    return;
  }

  if (!depth_sampler_->SetDepthImageAngles(
      _scan.angle_min(), _scan.angle_max(), _scan.vertical_angle_min(),
      _scan.vertical_angle_max()))
  {
    gzerr << "Invalid GPU lidar angular range on [" << gpu_topic_ << "].\n";
    return;
  }
  if (!depth_sampler_->ScanPatternFitsDepthImage())
  {
    if (!grid_error_reported_)
    {
      gzerr << "GPU lidar angular range on [" << gpu_topic_
             << "] does not cover scan pattern [" << csv_file_name_
             << "].\n";
      grid_error_reported_ = true;
    }
    return;
  }

  if (startup_scans_seen_++ < startup_skip_scans_)
    return;

  std::chrono::steady_clock::duration sim_time;
  size_t start_index;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_)
      return;

    sim_time = sim_time_;
    start_index = curr_start_index_;
    curr_start_index_ = (start_index + static_cast<size_t>(sample_step_)) %
      depth_sampler_->PointCount();
  }

  const auto results = depth_sampler_->SampleDepthImage(
    _scan.ranges().data(), width, height, start_index, sample_step_);
  if (results.empty())
    return;

  auto nanoseconds = _scan.header().stamp().sec() * 1000000000LL +
    _scan.header().stamp().nsec();
  if (nanoseconds == 0)
  {
    nanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(sim_time).count();
  }
  const rclcpp::Time stamp(nanoseconds, RCL_ROS_TIME);
  PublishPointCloud(results, stamp);
  rclcpp::spin_some(ros_node_);
}

void LivoxLidarSystem::PublishPointCloud(
  const std::vector<LivoxRayResult> &_results, const rclcpp::Time &_stamp)
{
  if (publish_pointcloud_type_ ==
      PointCloudType::LIVOX_LASER_SIMULATION_CUSTOM_MSG)
  {
    PublishLivoxCustomMsg(_results, _stamp);
    return;
  }
  PublishPointCloud2(_results, _stamp,
    publish_pointcloud_type_ ==
      PointCloudType::SENSOR_MSG_POINT_CLOUD2_LIVOXPOINTXYZRTLT);
}

void LivoxLidarSystem::PublishPointCloud2(
  const std::vector<LivoxRayResult> &_results, const rclcpp::Time &_stamp,
  bool _livox_fields)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = _stamp;
  cloud.header.frame_id = frame_name_;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(_results.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;

  AddField(cloud, "x", 0, sensor_msgs::msg::PointField::FLOAT32);
  AddField(cloud, "y", 4, sensor_msgs::msg::PointField::FLOAT32);
  AddField(cloud, "z", 8, sensor_msgs::msg::PointField::FLOAT32);
  if (_livox_fields)
  {
    AddField(cloud, "intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
    AddField(cloud, "tag", 16, sensor_msgs::msg::PointField::UINT8);
    AddField(cloud, "line", 17, sensor_msgs::msg::PointField::UINT8);
    AddField(cloud, "timestamp", 20, sensor_msgs::msg::PointField::FLOAT64);
    cloud.point_step = 28;
  }
  else
  {
    cloud.point_step = 12;
  }
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);

  const double base_time = _stamp.seconds();
  for (size_t i = 0; i < _results.size(); ++i)
  {
    const size_t base = i * cloud.point_step;
    const float x = static_cast<float>(_results[i].point.X());
    const float y = static_cast<float>(_results[i].point.Y());
    const float z = static_cast<float>(_results[i].point.Z());
    WriteValue(cloud.data, base, x);
    WriteValue(cloud.data, base + 4, y);
    WriteValue(cloud.data, base + 8, z);
    if (_livox_fields)
    {
      const float intensity = static_cast<float>(_results[i].intensity);
      const uint8_t tag = 0x10;
      const double timestamp = base_time + (publish_point_time_offsets_ ?
        static_cast<double>(_results[i].offset_time) * 1e-9 : 0.0);
      WriteValue(cloud.data, base + 12, intensity);
      WriteValue(cloud.data, base + 16, tag);
      WriteValue(cloud.data, base + 17, _results[i].line);
      WriteValue(cloud.data, base + 20, timestamp);
    }
  }
  pointcloud_pub_->publish(std::move(cloud));
}

void LivoxLidarSystem::PublishLivoxCustomMsg(
  const std::vector<LivoxRayResult> &_results, const rclcpp::Time &_stamp)
{
  msg::CustomMsg cloud;
  cloud.header.stamp = _stamp;
  cloud.header.frame_id = frame_name_;
  cloud.timebase = static_cast<uint64_t>(_stamp.nanoseconds());
  cloud.lidar_id = 0;
  cloud.rsvd = {0, 0, 0};
  cloud.points.reserve(_results.size());
  for (const auto &result : _results)
  {
    msg::CustomPoint point;
    point.offset_time = publish_point_time_offsets_ ? result.offset_time : 0u;
    point.x = static_cast<float>(result.point.X());
    point.y = static_cast<float>(result.point.Y());
    point.z = static_cast<float>(result.point.Z());
    point.reflectivity = static_cast<uint8_t>(std::clamp(result.intensity, 0.0, 255.0));
    point.tag = 0x10;
    point.line = result.line;
    cloud.points.push_back(point);
  }
  cloud.point_num = static_cast<uint32_t>(cloud.points.size());
  custom_msg_pub_->publish(std::move(cloud));
}

}  // namespace livox_laser_simulation

GZ_ADD_PLUGIN(livox_laser_simulation::LivoxLidarSystem,
              gz::sim::System,
              livox_laser_simulation::LivoxLidarSystem::ISystemConfigure,
              livox_laser_simulation::LivoxLidarSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(livox_laser_simulation::LivoxLidarSystem,
                    "livox_laser_simulation::LivoxLidarSystem")
