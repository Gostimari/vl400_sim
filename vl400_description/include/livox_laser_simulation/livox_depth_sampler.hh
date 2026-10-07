#ifndef LIVOX_LASER_SIMULATION_LIVOX_DEPTH_SAMPLER_HH_
#define LIVOX_LASER_SIMULATION_LIVOX_DEPTH_SAMPLER_HH_

#include <gz/math/Vector3.hh>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace livox_laser_simulation
{

struct ScanPoint
{
  double azimuth;
  double elevation;
  uint8_t line;
};

struct LivoxRayResult
{
  gz::math::Vector3d point;
  double intensity{0.0};
  uint32_t offset_time{0};
  uint8_t line{0};
};

class LivoxDepthSampler
{
public:
  bool LoadScanPattern(const std::string &_csv_file, int _line_count,
                       double _point_rate);
  void SetParameters(int _downsample, double _min_range, double _max_range,
                     bool _use_inf);
  bool SetDepthImageAngles(double _angle_min, double _angle_max,
                           double _vertical_angle_min,
                           double _vertical_angle_max);
  bool ScanPatternFitsDepthImage() const;

  std::vector<LivoxRayResult> SampleDepthImage(
    const double *_ranges, unsigned int _width, unsigned int _height,
    size_t _start_index, size_t _num_samples);

  size_t PointCount() const { return scan_points_.size(); }

private:
  std::vector<ScanPoint> scan_points_;
  int downsample_{1};
  double min_range_{0.1};
  double max_range_{40.0};
  double point_period_ns_{5000.0};
  double angle_min_{0.0};
  double angle_max_{0.0};
  double vertical_angle_min_{0.0};
  double vertical_angle_max_{0.0};
  double scan_angle_min_{0.0};
  double scan_angle_max_{0.0};
  double scan_elevation_min_{0.0};
  double scan_elevation_max_{0.0};
  bool depth_image_configured_{false};
  bool use_inf_{false};
};

}  // namespace livox_laser_simulation

#endif  // LIVOX_LASER_SIMULATION_LIVOX_DEPTH_SAMPLER_HH_
