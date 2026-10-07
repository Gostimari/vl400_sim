/*
 * TrackVisual: a gz-sim system that animates visual-only tread plates
 * riding the belt loop of a tracked vehicle.
 *
 * The belt is modeled as the convex hull of two circles (the two wheels
 * the belt wraps), inflated by a plate offset. Each tread plate hangs on
 * a polar 2-DOF chain:
 *
 *   track link --revolute(axis y, origin at loop center)--> arm link
 *              --prismatic(axis x)--> plate link (visual only)
 *
 * so plate position = center + r * (cos(psi), 0, -sin(psi)). Because the
 * loop is convex, the polar angle psi increases monotonically by exactly
 * 2*pi over one lap, which lets the revolute joint spin continuously and
 * the plates ride the whole loop without any wrap-around jumps.
 *
 * Plate motion is driven by the per-track velocity commands that
 * gz::sim::systems::TrackedVehicle publishes to each
 * gz::sim::systems::TrackController
 * (default topic /model/<model>/link/<link>/track_cmd_vel, gz.msgs.Double).
 * Joints are driven with JointVelocityCmd (feed-forward plus a small
 * position feedback term to prevent drift).
 */

#include <gz/plugin/Register.hh>

#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>

#include <sdf/Element.hh>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocityCmd.hh>

#include <gz/msgs/double.pb.h>

#include <gz/transport/Node.hh>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace vl400
{
namespace systems
{
inline namespace GZ_SIM_VERSION_NAMESPACE {

/// \brief Private data of the TrackVisual system.
struct TrackVisualPrivate
{
  /// \brief Model interface.
  gz::sim::Model model{gz::sim::kNullEntity};

  /// \brief Number of plates per track.
  std::size_t plateCount{20};

  /// \brief Outward offset of the plate path from the wheel circles [m].
  double beltOffset{0.03};

  /// \brief Position feedback gain.
  double gain{8.0};

  /// \brief Loop center (midpoint of the two wheel centers), x/z [m].
  double centerX{0.0};
  double centerZ{0.0};

  /// \brief Total loop length [m] and plate pitch [m].
  double loopLength{0.0};
  double pitch{0.0};

  /// \brief Number of arc-length samples in the tables.
  static constexpr std::size_t kSamples{2048};

  /// \brief psi(u) [rad, unwrapped over one lap] and r(u) [m] sampled
  /// uniformly in arc length u in [0, loopLength).
  std::vector<double> psiTab;
  std::vector<double> rTab;
  /// \brief Local derivatives dpsi/ds [rad/m] and dr/ds [1] per sample.
  std::vector<double> dpsiTab;
  std::vector<double> drTab;

  /// \brief Per-track velocity subscription callback. Each track gets its
  /// own instance so the callback knows which track it belongs to.
  struct TrackCallback
  {
    /// \brief Owner of the track list.
    TrackVisualPrivate *priv{nullptr};

    /// \brief Index of the track in TrackVisualPrivate::tracks.
    std::size_t idx{0};

    /// \brief Velocity callback.
    public: void OnVel(const gz::msgs::Double &_msg)
    {
      if (this->idx < this->priv->tracks.size())
      {
        this->priv->tracks[this->idx]->vel = _msg.data();
      }
    }
  };

  /// \brief Per-track state.
  struct Track
  {
    /// \brief Name of the track link (used for the default topic).
    std::string linkName;

    /// \brief Name prefix of the plate joints.
    std::string prefix;

    /// \brief Subscribed velocity [m/s] (written from transport thread).
    std::atomic<double> vel{0.0};

    /// \brief Unwrapped belt phase [m].
    double phase{0.0};

    /// \brief Velocity subscription callback object.
    std::unique_ptr<TrackCallback> callback;

    /// \brief Arm (revolute) joint entities, one per plate.
    std::vector<gz::sim::Entity> armJoints;

    /// \brief Slide (prismatic) joint entities, one per plate.
    std::vector<gz::sim::Entity> slideJoints;
  };

  /// \brief Configured tracks (left and right).
  std::vector<std::unique_ptr<Track>> tracks;

  /// \brief Transport node for the velocity subscriptions.
  gz::transport::Node node;
};

/// \brief A polygon point in the x-z plane.
struct LoopPoint
{
  double x;
  double z;
};

/// \brief A circle the belt wraps around (wheel center and belt radius),
/// given in the track link frame.
struct BeltCircle
{
  double x;
  double z;
  double r;
};

/// \brief External tangent between two circles. Returns the tangent
/// points and the unit normal pointing away from the belt interior
/// (upward for the upper hull, downward for the lower hull).
void CircleTangent(const BeltCircle &_a, const BeltCircle &_b, bool _upper,
                   LoopPoint &_pa, LoopPoint &_pb, double &_nx, double &_nz)
{
  const double dx = _b.x - _a.x;
  const double dz = _b.z - _a.z;
  const double d = std::hypot(dx, dz);
  const double k = (_b.r - _a.r) / d;
  const double s = std::sqrt(std::max(1.0 - k * k, 0.0));
  const double dhx = dx / d;
  const double dhz = dz / d;
  const double n1x = k * dhx - s * dhz;
  const double n1z = k * dhz + s * dhx;
  const double n2x = k * dhx + s * dhz;
  const double n2z = k * dhz - s * dhx;
  if (_upper ? (n1z > n2z) : (n1z < n2z))
  {
    _nx = n1x;
    _nz = n1z;
  }
  else
  {
    _nx = n2x;
    _nz = n2z;
  }
  _pa = {_a.x + _a.r * _nx, _a.z + _a.r * _nz};
  _pb = {_b.x + _b.r * _nx, _b.z + _b.r * _nz};
}

/// \brief Compute the belt loop polygon: the convex hull boundary of the
/// given circles, traversed such that the bottom segment moves toward -x
/// as the phase increases (belt surface moves backward when the robot
/// drives forward). The traversal is clockwise in the x-z plane and
/// starts at the bottom of the rightmost circle.
/// \param[in] _circles The belt circles (any order).
/// \param[in] _arcSamples Samples per wheel arc.
std::vector<LoopPoint> LoopPolygon(
    const std::vector<BeltCircle> &_circles, std::size_t _arcSamples)
{
  std::vector<BeltCircle> c = _circles;
  std::sort(c.begin(), c.end(),
            [](const BeltCircle &a, const BeltCircle &b)
            {
              return a.x < b.x;
            });
  if (c.size() < 2)
  {
    gzerr << "TrackVisual: the belt needs at least two wheels."
          << std::endl;
    return {};
  }

  // Monotone chain over disks: indices into c, ascending x. A middle
  // circle is excludable iff it lies entirely on the interior side of
  // the tangent between its neighbors (signed distance along the
  // outward normal <= -radius).
  auto buildChain = [&c](bool _upper)
  {
    std::vector<std::size_t> chain;
    for (std::size_t i = 0; i < c.size(); ++i)
    {
      while (chain.size() >= 2)
      {
        const auto &a = c[chain[chain.size() - 2]];
        const auto &b = c[chain.back()];
        LoopPoint pa, pb;
        double nx, nz;
        CircleTangent(a, c[i], _upper, pa, pb, nx, nz);
        const double f = (b.x - pa.x) * nx + (b.z - pa.z) * nz;
        if (f <= -b.r + 1e-9)
        {
          chain.pop_back();
        }
        else
        {
          break;
        }
      }
      chain.push_back(i);
    }
    return chain;
  };

  const auto lower = buildChain(false);
  const auto upper = buildChain(true);

  std::vector<LoopPoint> pts;
  pts.reserve(2 * _arcSamples * (c.size() + 2) + 8);

  // Unwrapped, decreasing standard angle of the traversal. Starts at
  // the bottom of the rightmost circle (-pi/2) and decreases by 2*pi
  // over one lap.
  double angle = -0.5 * GZ_PI;
  const auto &right = c[lower.back()];
  pts.push_back({right.x, right.z - right.r});

  // Append the clockwise arc on _circ from the current angle down to the
  // wrapped angle _theta, then continue from there.
  auto appendArc = [&](const BeltCircle &_circ, double _theta)
  {
    double target = _theta;
    while (target > angle)
    {
      target -= 2.0 * GZ_PI;
    }
    const double span = angle - target;
    for (std::size_t k = 1; k <= _arcSamples; ++k)
    {
      const double a = angle - span * static_cast<double>(k) / _arcSamples;
      pts.push_back({_circ.x + _circ.r * std::cos(a),
                     _circ.z + _circ.r * std::sin(a)});
    }
    angle = target;
  };

  // Lower hull, right to left.
  for (std::size_t k = lower.size() - 1; k > 0; --k)
  {
    const auto &a = c[lower[k]];
    const auto &b = c[lower[k - 1]];
    LoopPoint pa, pb;
    double nx, nz;
    CircleTangent(a, b, false, pa, pb, nx, nz);
    appendArc(a, std::atan2(nz, nx));
    pts.push_back(pb);
  }

  // Upper hull, left to right.
  for (std::size_t k = 0; k + 1 < upper.size(); ++k)
  {
    const auto &a = c[upper[k]];
    const auto &b = c[upper[k + 1]];
    LoopPoint pa, pb;
    double nx, nz;
    CircleTangent(a, b, true, pa, pb, nx, nz);
    appendArc(a, std::atan2(nz, nx));
    pts.push_back(pb);
  }

  // Closing arc around the front of the rightmost circle, down to the
  // starting point one lap below.
  const auto &rightUpper = c[upper.back()];
  appendArc(rightUpper, -0.5 * GZ_PI - 2.0 * GZ_PI);

  return pts;
}

/// \brief The TrackVisual system.
class TrackVisual
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
{
  /// \brief Constructor.
  public: TrackVisual() : dataPtr(std::make_unique<TrackVisualPrivate>())
  {
  }

  /// \brief Destructor.
  public: ~TrackVisual() override = default;

  // Documentation inherited.
  public: void Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &) override;

  // Documentation inherited.
  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Build the uniform arc-length tables from the loop polygon.
  private: void BuildTables(const std::vector<LoopPoint> &_pts);

  /// \brief Sample the tables at arc length u in [0, loopLength).
  /// \param[out] _psi Unwrapped polar angle.
  /// \param[out] _r Radial distance.
  /// \param[out] _dpsi Local dpsi/ds.
  /// \param[out] _dr Local dr/ds.
  private: void Sample(double _u, double &_psi, double &_r,
                       double &_dpsi, double &_dr) const;

  /// \brief Resolve the joint entities of one track (retry until found).
  /// \return True once all joints have been resolved.
  private: bool ResolveJoints(TrackVisualPrivate::Track &_track,
                              gz::sim::EntityComponentManager &_ecm);

  /// \brief Read the first DoF of a joint position, or a given fallback.
  private: static double JointPosOr(
      gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _joint,
      double _fallback);

  /// \brief Private data pointer.
  private: gz::sim::Entity modelEntity;

  private: std::unique_ptr<TrackVisualPrivate> dataPtr;
};

void TrackVisual::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  this->modelEntity = _entity;
  this->dataPtr->model = gz::sim::Model(_entity);
  const auto modelName = this->dataPtr->model.Name(_ecm);
  if (modelName.empty())
  {
    gzerr << "TrackVisual must be attached to a model entity."
          << std::endl;
    return;
  }

  // Belt circles: repeated <wheel> elements with (x, z, r) triples in
  // the track link frame (any order). Defaults match the vl400: rear
  // visual wheel, main drive wheel and front support wheel, with radii
  // including the 1 cm belt thickness.
  std::vector<BeltCircle> circles;
  auto wheelElem = _sdf->FindElement("wheel");
  while (wheelElem != nullptr)
  {
    const auto w = wheelElem->Get<gz::math::Vector3d>(
        "", gz::math::Vector3d::Zero).first;
    if (w.Z() <= 0.0)
    {
      gzerr << "TrackVisual: <wheel> needs (x, z, r) with r > 0."
            << std::endl;
      return;
    }
    circles.push_back({w.X(), w.Y(), w.Z()});
    wheelElem = wheelElem->GetNextElement("wheel");
  }
  if (circles.empty())
  {
    // Backwards compatibility with the old <wheel1>/<wheel2> pair.
    const auto w1 = _sdf->Get<gz::math::Vector3d>(
        "wheel1", gz::math::Vector3d(1.005, 0.501, 0.46)).first;
    const auto w2 = _sdf->Get<gz::math::Vector3d>(
        "wheel2", gz::math::Vector3d(1.755, 0.25, 0.21)).first;
    circles.push_back({w1.X(), w1.Y(), w1.Z()});
    circles.push_back({w2.X(), w2.Y(), w2.Z()});
  }

  this->dataPtr->beltOffset = _sdf->Get<double>("belt_offset", 0.03).first;
  this->dataPtr->plateCount = static_cast<std::size_t>(
      _sdf->Get<unsigned int>("plate_count", 20).first);
  this->dataPtr->gain = _sdf->Get<double>("gain", 8.0).first;
  if (this->dataPtr->plateCount == 0)
  {
    gzerr << "TrackVisual: <plate_count> must be positive." << std::endl;
    return;
  }

  // The plate path rides <belt_offset> outside the belt surface; the
  // wheel radii passed in already include the belt.
  for (auto &circle : circles)
  {
    circle.r += this->dataPtr->beltOffset;
  }
  const auto pts = LoopPolygon(circles, 32);
  if (pts.empty())
  {
    return;
  }

  // Polar center of the plate arms: centroid of the wheel centers
  // (inside the convex hull, so the polar angle is monotonic).
  double cx = 0.0;
  double cz = 0.0;
  for (const auto &circle : circles)
  {
    cx += circle.x;
    cz += circle.z;
  }
  this->dataPtr->centerX = cx / static_cast<double>(circles.size());
  this->dataPtr->centerZ = cz / static_cast<double>(circles.size());
  this->BuildTables(pts);
  if (this->dataPtr->loopLength <= 0.0)
  {
    return;
  }
  this->dataPtr->pitch = this->dataPtr->loopLength /
      static_cast<double>(this->dataPtr->plateCount);

  // Per-track configuration.
  auto trackElem = _sdf->FindElement("track");
  while (trackElem != nullptr)
  {
    auto &track = *(this->dataPtr->tracks.emplace_back(
        std::make_unique<TrackVisualPrivate::Track>()));
    track.linkName = trackElem->Get<std::string>("link", "").first;
    track.prefix = trackElem->Get<std::string>("prefix", "").first;
    if (track.linkName.empty() || track.prefix.empty())
    {
      gzerr << "TrackVisual: <track> requires <link> and <prefix>."
            << std::endl;
      return;
    }
    const auto defaultTopic = "/model/" + modelName + "/link/" +
        track.linkName + "/track_cmd_vel";
    const auto topic = trackElem->Get<std::string>(
        "velocity_topic", defaultTopic).first;

    const auto idx = this->dataPtr->tracks.size() - 1;
    track.callback = std::make_unique<TrackVisualPrivate::TrackCallback>();
    track.callback->priv = this->dataPtr.get();
    track.callback->idx = idx;
    if (!this->dataPtr->node.Subscribe(topic,
        &TrackVisualPrivate::TrackCallback::OnVel, track.callback.get()))
    {
      gzerr << "TrackVisual: error subscribing to [" << topic << "]."
            << std::endl;
      return;
    }
    gzmsg << "TrackVisual: track [" << track.prefix << "] subscribing to ["
          << topic << "]" << std::endl;
    trackElem = trackElem->GetNextElement("track");
  }

  if (this->dataPtr->tracks.empty())
  {
    gzerr << "TrackVisual: no <track> elements configured." << std::endl;
    return;
  }

  gzmsg << "TrackVisual: model [" << modelName << "] loop length ["
        << this->dataPtr->loopLength << " m], plates/track ["
        << this->dataPtr->plateCount << "], pitch ["
        << this->dataPtr->pitch << " m]" << std::endl;
}

void TrackVisual::BuildTables(const std::vector<LoopPoint> &_pts)
{
  const auto n = _pts.size();
  if (n < 3)
  {
    return;
  }

  // Cumulative arc length over the closed polygon.
  std::vector<double> cum(n, 0.0);
  double total = 0.0;
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto &p = _pts[i];
    const auto &q = _pts[(i + 1) % n];
    total += std::hypot(q.x - p.x, q.z - p.z);
    cum[i] = total;
  }
  this->dataPtr->loopLength = total;
  this->dataPtr->psiTab.assign(this->dataPtr->kSamples, 0.0);
  this->dataPtr->rTab.assign(this->dataPtr->kSamples, 0.0);
  this->dataPtr->dpsiTab.assign(this->dataPtr->kSamples, 0.0);
  this->dataPtr->drTab.assign(this->dataPtr->kSamples, 0.0);

  // Raw polar coordinates of the vertices; psi measured so that a plate at
  // psi has position center + r * (cos psi, 0, -sin psi).
  auto polarPsi = [this](const LoopPoint &_p)
  {
    return std::atan2(-(_p.z - this->dataPtr->centerZ),
                      _p.x - this->dataPtr->centerX);
  };
  auto polarR = [this](const LoopPoint &_p)
  {
    return std::hypot(_p.x - this->dataPtr->centerX,
                      _p.z - this->dataPtr->centerZ);
  };

  // Unwrap psi monotonically along the traversal.
  std::vector<double> psiRaw(n, 0.0);
  double prev = 0.0;
  bool first = true;
  for (std::size_t i = 0; i < n; ++i)
  {
    double psi = polarPsi(_pts[i]);
    if (!first)
    {
      double d = std::fmod(psi - prev, 2.0 * GZ_PI);
      if (d < 0.0)
      {
        d += 2.0 * GZ_PI;
      }
      if (d > GZ_PI)
      {
        d -= 2.0 * GZ_PI;
      }
      psi = prev + d;
    }
    psiRaw[i] = psi;
    prev = psi;
    first = false;
  }

  // Walk the polygon sampling uniformly in arc length.
  std::size_t seg = 0;
  for (std::size_t k = 0; k < this->dataPtr->kSamples; ++k)
  {
    const double u = total * static_cast<double>(k) / this->dataPtr->kSamples;
    while (seg + 1 < n && cum[seg] < u)
    {
      ++seg;
    }
    const double segStart = (seg == 0) ? 0.0 : cum[seg - 1];
    const double segLen = cum[seg] - segStart;
    double frac = (segLen > 1e-12) ? (u - segStart) / segLen : 0.0;
    frac = std::min(std::max(frac, 0.0), 1.0);
    const auto &p = _pts[seg];
    const auto &q = _pts[(seg + 1) % n];
    const LoopPoint pt{p.x + frac * (q.x - p.x), p.z + frac * (q.z - p.z)};

    double psi = polarPsi(pt);
    // Keep psi continuous with the vertex unwrapping.
    double ref = psiRaw[seg] + frac *
        (psiRaw[(seg + 1) % n] +
         ((seg + 1 == n) ? 2.0 * GZ_PI : 0.0) - psiRaw[seg]);
    double d = std::fmod(psi - ref, 2.0 * GZ_PI);
    if (d > GZ_PI)
    {
      d -= 2.0 * GZ_PI;
    }
    if (d < -GZ_PI)
    {
      d += 2.0 * GZ_PI;
    }
    this->dataPtr->psiTab[k] = ref + d;
    this->dataPtr->rTab[k] = polarR(pt);
  }

  // Re-reference psiTab so it starts at its raw (wrapped) value; the
  // samples must be increasing with total span 2*pi.
  double base = this->dataPtr->psiTab[0];
  this->dataPtr->psiTab[0] = std::fmod(base, 2.0 * GZ_PI);
  for (std::size_t k = 1; k < this->dataPtr->kSamples; ++k)
  {
    this->dataPtr->psiTab[k] -= base - this->dataPtr->psiTab[0];
  }
  // Force monotonicity against tiny numeric jitter.
  for (std::size_t k = 1; k < this->dataPtr->kSamples; ++k)
  {
    if (this->dataPtr->psiTab[k] <= this->dataPtr->psiTab[k - 1])
    {
      this->dataPtr->psiTab[k] = this->dataPtr->psiTab[k - 1] + 1e-9;
    }
  }

  // Local derivatives (per meter of path).
  const double ds = total / this->dataPtr->kSamples;
  for (std::size_t k = 0; k < this->dataPtr->kSamples; ++k)
  {
    const std::size_t kNext = (k + 1) % this->dataPtr->kSamples;
    const std::size_t kPrev =
        (k == 0) ? this->dataPtr->kSamples - 1 : k - 1;
    double dPsi = this->dataPtr->psiTab[kNext] - this->dataPtr->psiTab[k];
    if (kNext == 0)
    {
      dPsi += 2.0 * GZ_PI;
    }
    double dPsiPrev = this->dataPtr->psiTab[k] - this->dataPtr->psiTab[kPrev];
    if (k == 0)
    {
      dPsiPrev += 2.0 * GZ_PI;
    }
    this->dataPtr->dpsiTab[k] = 0.5 * (dPsi + dPsiPrev) / ds;
    const double dR = this->dataPtr->rTab[kNext] - this->dataPtr->rTab[k];
    const double dRPrev = this->dataPtr->rTab[k] - this->dataPtr->rTab[kPrev];
    this->dataPtr->drTab[k] = 0.5 * (dR + dRPrev) / ds;
  }
}

void TrackVisual::Sample(double _u, double &_psi, double &_r,
                         double &_dpsi, double &_dr) const
{
  const auto m = this->dataPtr->kSamples;
  double f = _u / this->dataPtr->loopLength * static_cast<double>(m);
  if (f < 0.0)
  {
    f = 0.0;
  }
  if (f >= static_cast<double>(m))
  {
    f = static_cast<double>(m) - 1e-9;
  }
  const auto i0 = static_cast<std::size_t>(f);
  const auto i1 = (i0 + 1) % m;
  const double frac = f - static_cast<double>(i0);
  const double psi1 = (i1 == 0) ?
      this->dataPtr->psiTab[i0] +
      (this->dataPtr->psiTab[0] + 2.0 * GZ_PI - this->dataPtr->psiTab[i0]) :
      this->dataPtr->psiTab[i1];
  _psi = this->dataPtr->psiTab[i0] +
      frac * (psi1 - this->dataPtr->psiTab[i0]);
  _r = this->dataPtr->rTab[i0] +
      frac * (this->dataPtr->rTab[i1] - this->dataPtr->rTab[i0]);
  _dpsi = this->dataPtr->dpsiTab[i0];
  _dr = this->dataPtr->drTab[i0];
}

bool TrackVisual::ResolveJoints(
    TrackVisualPrivate::Track &_track, gz::sim::EntityComponentManager &_ecm)
{
  if (!_track.armJoints.empty())
  {
    return true;
  }
  _track.armJoints.assign(this->dataPtr->plateCount, gz::sim::kNullEntity);
  _track.slideJoints.assign(this->dataPtr->plateCount, gz::sim::kNullEntity);
  for (std::size_t i = 0; i < this->dataPtr->plateCount; ++i)
  {
    const auto arm = this->dataPtr->model.JointByName(
        _ecm, _track.prefix + "_arm_j_" + std::to_string(i));
    const auto slide = this->dataPtr->model.JointByName(
        _ecm, _track.prefix + "_slide_j_" + std::to_string(i));
    if (arm == gz::sim::kNullEntity || slide == gz::sim::kNullEntity)
    {
      static std::size_t warned{0};
      if (warned++ % 500 == 0)
      {
        gzwarn << "TrackVisual: waiting for plate joints ["
               << _track.prefix << "_arm_j_" << i << "]..." << std::endl;
      }
      _track.armJoints.clear();
      _track.slideJoints.clear();
      return false;
    }
    _track.armJoints[i] = arm;
    _track.slideJoints[i] = slide;

    // Ask physics to report the position of these joints. The physics
    // system only fills JointPosition for joints that have the component.
    for (const auto joint : {arm, slide})
    {
      if (_ecm.Component<gz::sim::components::JointPosition>(joint) ==
          nullptr)
      {
        _ecm.CreateComponent(
            joint, gz::sim::components::JointPosition());
      }
    }
  }
  gzmsg << "TrackVisual: found " << this->dataPtr->plateCount
        << " plate joints for track [" << _track.prefix << "]."
        << std::endl;
  return true;
}

double TrackVisual::JointPosOr(
    gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _joint,
    double _fallback)
{
  auto pos = _ecm.Component<gz::sim::components::JointPosition>(_joint);
  if (pos == nullptr || pos->Data().empty())
  {
    return _fallback;
  }
  return pos->Data()[0];
}

void TrackVisual::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
  {
    return;
  }
  const double dt = std::chrono::duration<double>(_info.dt).count();
  if (dt <= 0.0)
  {
    return;
  }

  for (const auto &trackPtr : this->dataPtr->tracks)
  {
    auto &track = *trackPtr;
    if (!this->ResolveJoints(track, _ecm))
    {
      continue;
    }

    const double vel = track.vel;
    track.phase += vel * dt;

    for (std::size_t i = 0; i < this->dataPtr->plateCount; ++i)
    {
      const double s = track.phase +
          static_cast<double>(i) * this->dataPtr->pitch;
      const double laps = std::floor(s / this->dataPtr->loopLength);
      const double u = s - laps * this->dataPtr->loopLength;

      double psi, r, dpsi, dr;
      this->Sample(u, psi, r, dpsi, dr);
      const double psiTarget = psi + 2.0 * GZ_PI * laps;

      const double psiActual = JointPosOr(
          _ecm, track.armJoints[i], psiTarget);
      const double rActual = JointPosOr(
          _ecm, track.slideJoints[i], r);

      // Feed-forward plus position feedback.
      double vArm = dpsi * vel +
          this->dataPtr->gain * (psiTarget - psiActual);
      vArm = std::min(std::max(vArm, -30.0), 30.0);
      double vSlide = dr * vel +
          this->dataPtr->gain * (r - rActual);
      vSlide = std::min(std::max(vSlide, -10.0), 10.0);

      _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(
          track.armJoints[i], {vArm});
      _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(
          track.slideJoints[i], {vSlide});
    }
  }
}

}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace systems
}  // namespace vl400

// Register the system.
GZ_ADD_PLUGIN(vl400::systems::TrackVisual,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

// Add plugin alias so the SDF <name> attribute can be the short name.
GZ_ADD_PLUGIN_ALIAS(vl400::systems::TrackVisual, "TrackVisual")
GZ_ADD_PLUGIN_ALIAS(vl400::systems::TrackVisual, "vl400::systems::TrackVisual")
