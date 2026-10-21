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
    // Bilinear range lookup at the exact pattern direction. The depth
    // image is a coarse range grid (0.5 deg/pixel); interpolating keeps
    // the range error comparable to a nearest-pixel lookup on a fine
    // grid. Non-finite or out-of-range corners (misses) are dropped and
    // the weights renormalized, so depth edges stay crisp.
    double fx = horizontal_ratio * static_cast<double>(_width - 1);
    double fy = vertical_ratio * static_cast<double>(_height - 1);
    if (fx < 0.0)
      fx = 0.0;
    if (fy < 0.0)
      fy = 0.0;
    if (fx > static_cast<double>(_width - 1))
      fx = static_cast<double>(_width - 1);
    if (fy > static_cast<double>(_height - 1))
      fy = static_cast<double>(_height - 1);
    const auto x0 = static_cast<unsigned int>(fx);
    const auto y0 = static_cast<unsigned int>(fy);
    const auto x1 = std::min(x0 + 1, _width - 1);
    const auto y1 = std::min(y0 + 1, _height - 1);
    const double tx = fx - static_cast<double>(x0);
    const double ty = fy - static_cast<double>(y0);
    const double weights[4] = {
      (1.0 - tx) * (1.0 - ty), tx * (1.0 - ty),
      (1.0 - tx) * ty, tx * ty};
    const size_t pixels[4] = {
      static_cast<size_t>(y0) * _width + x0,
      static_cast<size_t>(y0) * _width + x1,
      static_cast<size_t>(y1) * _width + x0,
      static_cast<size_t>(y1) * _width + x1};

    double range = 0.0;
    double weight_sum = 0.0;
    for (int c = 0; c < 4; ++c)
    {
      const double corner = _ranges[pixels[c]];
      if (std::isfinite(corner) && corner >= min_range_ &&
          corner <= max_range_)
      {
        range += weights[c] * corner;
        weight_sum += weights[c];
      }
    }
    const bool hit = weight_sum > 0.0;
    if (!hit && !use_inf_)
      continue;
    if (!hit)
      range = max_range_;
    else
      range /= weight_sum;

    // Emit the point at the exact scan-pattern direction. Using the
    // pixel-grid angle instead would quantize every direction to the
    // depth-image resolution (0.2 deg here): many pattern points would
    // collapse onto the same pixel direction and the fine structure of
    // the measured HAP pattern would be destroyed. The depth image is
    // only used to look up the range.
    const double cos_elevation = std::cos(scan.elevation);

    LivoxRayResult result;
    result.point.Set(range * cos_elevation * std::cos(azimuth),
                     range * cos_elevation * std::sin(azimuth),
                     range * std::sin(scan.elevation));
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
