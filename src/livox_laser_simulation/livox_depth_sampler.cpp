#include "livox_laser_simulation/livox_depth_sampler.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "livox_laser_simulation/csv_reader.hpp"

namespace livox_laser_simulation
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double WrapAngle(double _angle, double _minimum, double _maximum)
{
  if (_maximum - _minimum < kTwoPi - 1e-6)
    return _angle;

  while (_angle < _minimum)
    _angle += kTwoPi;
  while (_angle > _maximum)
    _angle -= kTwoPi;
  return _angle;
}
}  // namespace

bool LivoxDepthSampler::LoadScanPattern(const std::string &_csv_file,
                                        int _line_count, double _point_rate)
{
  if (_line_count < 1 || _line_count > 256 || _point_rate <= 0.0)
    return false;

  std::vector<std::vector<double>> data;
  if (!CsvReader::ReadCsvFile(_csv_file, data))
    return false;

  scan_points_.clear();
  scan_points_.reserve(data.size());
  scan_angle_min_ = std::numeric_limits<double>::max();
  scan_angle_max_ = std::numeric_limits<double>::lowest();
  scan_elevation_min_ = std::numeric_limits<double>::max();
  scan_elevation_max_ = std::numeric_limits<double>::lowest();
  constexpr double deg_to_rad = kPi / 180.0;
  for (size_t i = 0; i < data.size(); ++i)
  {
    if (data[i].size() != 3)
      continue;

    const double pitch = data[i][2] * deg_to_rad - kPi / 2.0;
    const double azimuth = data[i][1] * deg_to_rad;
    const double elevation = -pitch;
    scan_points_.push_back({azimuth, elevation,
      static_cast<uint8_t>(i % static_cast<size_t>(_line_count))});
    scan_angle_min_ = std::min(scan_angle_min_, azimuth);
    scan_angle_max_ = std::max(scan_angle_max_, azimuth);
    scan_elevation_min_ = std::min(scan_elevation_min_, elevation);
    scan_elevation_max_ = std::max(scan_elevation_max_, elevation);
  }
  point_period_ns_ = 1e9 / _point_rate;
  return !scan_points_.empty();
}

void LivoxDepthSampler::SetParameters(int _downsample, double _min_range,
                                      double _max_range, bool _use_inf)
{
  downsample_ = std::max(1, _downsample);
  min_range_ = _min_range;
  max_range_ = _max_range;
  use_inf_ = _use_inf;
}

bool LivoxDepthSampler::SetDepthImageAngles(
  double _angle_min, double _angle_max, double _vertical_angle_min,
  double _vertical_angle_max)
{
  if (_angle_max <= _angle_min ||
      _vertical_angle_max <= _vertical_angle_min)
  {
    return false;
  }

  angle_min_ = _angle_min;
  angle_max_ = _angle_max;
  vertical_angle_min_ = _vertical_angle_min;
  vertical_angle_max_ = _vertical_angle_max;
  depth_image_configured_ = true;
  return true;
}

bool LivoxDepthSampler::ScanPatternFitsDepthImage() const
{
  if (!depth_image_configured_ || scan_points_.empty())
    return false;

  const bool full_horizontal_view =
    angle_max_ - angle_min_ >= kTwoPi - 1e-6;
  const bool horizontal_fits = full_horizontal_view ||
    (scan_angle_min_ >= angle_min_ - 1e-6 &&
     scan_angle_max_ <= angle_max_ + 1e-6);
  const bool vertical_fits =
    scan_elevation_min_ >= vertical_angle_min_ - 1e-6 &&
    scan_elevation_max_ <= vertical_angle_max_ + 1e-6;
  return horizontal_fits && vertical_fits;
}

std::vector<LivoxRayResult> LivoxDepthSampler::SampleDepthImage(
  const double *_ranges, unsigned int _width, unsigned int _height,
  size_t _start_index, size_t _num_samples)
{
  std::vector<LivoxRayResult> results;
  if (!_ranges || _width < 2 || _height < 2 || !depth_image_configured_ ||
      scan_points_.empty())
  {
    return results;
  }

  results.reserve((_num_samples + downsample_ - 1) / downsample_);
  std::array<size_t, 256> line_sample_counts{};
  const size_t downsample = static_cast<size_t>(downsample_);
  for (size_t block_start = 0; block_start < _num_samples;
       block_start += downsample)
  {
    const size_t block_end = std::min(block_start + downsample, _num_samples);
    size_t offset = block_start;
    size_t best_count = std::numeric_limits<size_t>::max();
    for (size_t candidate = block_start; candidate < block_end; ++candidate)
    {
      const auto &candidate_scan =
        scan_points_[(_start_index + candidate) % scan_points_.size()];
      const size_t count = line_sample_counts[candidate_scan.line];
      if (count < best_count)
      {
        offset = candidate;
        best_count = count;
      }
    }

    const auto &scan =
      scan_points_[(_start_index + offset) % scan_points_.size()];
    ++line_sample_counts[scan.line];

    const double azimuth = WrapAngle(scan.azimuth, angle_min_, angle_max_);
    if (azimuth < angle_min_ || azimuth > angle_max_ ||
        scan.elevation < vertical_angle_min_ ||
        scan.elevation > vertical_angle_max_)
    {
      continue;
    }

    const double horizontal_ratio =
      (azimuth - angle_min_) / (angle_max_ - angle_min_);
    const double vertical_ratio =
      (scan.elevation - vertical_angle_min_) /
      (vertical_angle_max_ - vertical_angle_min_);
    const auto column = static_cast<unsigned int>(std::llround(
      horizontal_ratio * static_cast<double>(_width - 1)));
    const auto row = static_cast<unsigned int>(std::llround(
      vertical_ratio * static_cast<double>(_height - 1)));
    const size_t pixel = static_cast<size_t>(row) * _width + column;

    double range = _ranges[pixel];
    const bool hit = std::isfinite(range) && range >= min_range_ &&
      range <= max_range_;
    if (!hit && !use_inf_)
      continue;
    if (!hit)
      range = max_range_;

    const double sampled_azimuth = angle_min_ +
      static_cast<double>(column) / static_cast<double>(_width - 1) *
      (angle_max_ - angle_min_);
    const double sampled_elevation = vertical_angle_min_ +
      static_cast<double>(row) / static_cast<double>(_height - 1) *
      (vertical_angle_max_ - vertical_angle_min_);
    const double cos_elevation = std::cos(sampled_elevation);

    LivoxRayResult result;
    result.point.Set(range * cos_elevation * std::cos(sampled_azimuth),
                     range * cos_elevation * std::sin(sampled_azimuth),
                     range * std::sin(sampled_elevation));
    result.intensity = hit ? 100.0 : 0.0;
    result.offset_time = static_cast<uint32_t>(std::min(
      point_period_ns_ * static_cast<double>(offset),
      static_cast<double>(std::numeric_limits<uint32_t>::max())));
    result.line = scan.line;
    results.push_back(result);
  }
  return results;
}

}  // namespace livox_laser_simulation
