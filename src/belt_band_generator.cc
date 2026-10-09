/*
 * Generates belt_band.obj: a continuous, closed track band wrapped
 * around the three belt wheels of the vl400, as a single mesh.
 *
 * The band is the convex hull of the belt circles, extruded laterally
 * (y in [-w/2, w/2]) and radially inwards by the band thickness. It is
 * attached statically to the track links; only the cleats animate.
 *
 * Coordinates are the track link frame, matching the <wheel> elements
 * of the TrackVisual plugin in urdf/vl400.xacro.
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;

struct Circle
{
  double x;
  double z;
  double r;
};

struct Point
{
  double x;
  double z;
};

// Belt circles (x, z, r) in the track link frame. Radii include the
// 3 cm plate offset: the band surface is exactly where the animated
// cleats ride.
const std::vector<Circle> kCircles = {
  {0.555, 0.501, 0.49},   // rear visual wheel
  {1.005, 0.501, 0.49},   // main drive wheel
  {1.755, 0.25, 0.24},    // front support wheel
};

constexpr double kHalfWidth = 0.15;   // band lateral half width [m]
constexpr double kThickness = 0.025;  // band radial thickness [m]
constexpr std::size_t kArcSamples = 32;

void CircleTangent(const Circle &a, const Circle &b, bool upper,
                   Point &pa, Point &pb, double &nx, double &nz)
{
  const double dx = b.x - a.x;
  const double dz = b.z - a.z;
  const double d = std::hypot(dx, dz);
  const double k = (b.r - a.r) / d;
  const double s = std::sqrt(std::max(1.0 - k * k, 0.0));
  const double dhx = dx / d;
  const double dhz = dz / d;
  const double n1x = k * dhx - s * dhz;
  const double n1z = k * dhz + s * dhx;
  const double n2x = k * dhx + s * dhz;
  const double n2z = k * dhz - s * dhx;
  if (upper ? (n1z > n2z) : (n1z < n2z))
  {
    nx = n1x;
    nz = n1z;
  }
  else
  {
    nx = n2x;
    nz = n2z;
  }
  pa = {a.x + a.r * nx, a.z + a.r * nz};
  pb = {b.x + b.r * nx, b.z + b.r * nz};
}

// Convex hull boundary of the circles, traversed clockwise in the x-z
// plane starting at the bottom of the rightmost circle (same traversal
// as the TrackVisual system, so cleats and band share the same loop).
std::vector<Point> LoopPolygon()
{
  std::vector<Circle> c = kCircles;
  std::sort(c.begin(), c.end(),
            [](const Circle &a, const Circle &b) { return a.x < b.x; });

  auto buildChain = [&c](bool upper)
  {
    std::vector<std::size_t> chain;
    for (std::size_t i = 0; i < c.size(); ++i)
    {
      while (chain.size() >= 2)
      {
        const auto &a = c[chain[chain.size() - 2]];
        const auto &b = c[chain.back()];
        Point pa, pb;
        double nx, nz;
        CircleTangent(a, c[i], upper, pa, pb, nx, nz);
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

  std::vector<Point> pts;
  double angle = -0.5 * kPi;
  const auto &right = c[lower.back()];
  pts.push_back({right.x, right.z - right.r});

  auto appendArc = [&](const Circle &circ, double theta)
  {
    double target = theta;
    while (target > angle)
    {
      target -= 2.0 * kPi;
    }
    const double span = angle - target;
    for (std::size_t k = 1; k <= kArcSamples; ++k)
    {
      const double a = angle - span * static_cast<double>(k) / kArcSamples;
      pts.push_back({circ.x + circ.r * std::cos(a),
                     circ.z + circ.r * std::sin(a)});
    }
    angle = target;
  };

  for (std::size_t k = lower.size() - 1; k > 0; --k)
  {
    const auto &a = c[lower[k]];
    const auto &b = c[lower[k - 1]];
    Point pa, pb;
    double nx, nz;
    CircleTangent(a, b, false, pa, pb, nx, nz);
    appendArc(a, std::atan2(nz, nx));
    pts.push_back(pb);
  }

  for (std::size_t k = 0; k + 1 < upper.size(); ++k)
  {
    const auto &a = c[upper[k]];
    const auto &b = c[upper[k + 1]];
    Point pa, pb;
    double nx, nz;
    CircleTangent(a, b, true, pa, pb, nx, nz);
    appendArc(a, std::atan2(nz, nx));
    pts.push_back(pb);
  }

  const auto &rightUpper = c[upper.back()];
  appendArc(rightUpper, -0.5 * kPi - 2.0 * kPi);

  return pts;
}

}  // namespace

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::fprintf(stderr, "usage: %s <output.dae>\n", argv[0]);
    return 1;
  }

  const auto loop = LoopPolygon();
  if (loop.size() < 3)
  {
    std::fprintf(stderr, "belt_band_generator: loop generation failed\n");
    return 1;
  }

  // Polar center used for the inward offset of the inner ring and for
  // orienting the segment normals.
  double cx = 0.0;
  double cz = 0.0;
  for (const auto &circle : kCircles)
  {
    cx += circle.x;
    cz += circle.z;
  }
  cx /= static_cast<double>(kCircles.size());
  cz /= static_cast<double>(kCircles.size());

  // Corner vertex positions, indexed 0-based:
  // outer[i] @ +y : 4*i + 0, outer[i] @ -y : 4*i + 1,
  // inner[i] @ +y : 4*i + 2, inner[i] @ -y : 4*i + 3.
  const std::size_t n = loop.size();
  std::vector<double> vx(4 * n);
  std::vector<double> vy(4 * n);
  std::vector<double> vz(4 * n);
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto &p = loop[i];
    double dx = p.x - cx;
    double dz = p.z - cz;
    const double d = std::hypot(dx, dz);
    if (d > 1e-9)
    {
      dx /= d;
      dz /= d;
    }
    const double ix = p.x - kThickness * dx;
    const double iz = p.z - kThickness * dz;
    vx[4 * i + 0] = p.x;  vy[4 * i + 0] = kHalfWidth;  vz[4 * i + 0] = p.z;
    vx[4 * i + 1] = p.x;  vy[4 * i + 1] = -kHalfWidth; vz[4 * i + 1] = p.z;
    vx[4 * i + 2] = ix;   vy[4 * i + 2] = kHalfWidth;  vz[4 * i + 2] = iz;
    vx[4 * i + 3] = ix;   vy[4 * i + 3] = -kHalfWidth; vz[4 * i + 3] = iz;
  }

  auto oyp = [&](std::size_t i) { return 4 * (i % n) + 0; };
  auto oym = [&](std::size_t i) { return 4 * (i % n) + 1; };
  auto iyp = [&](std::size_t i) { return 4 * (i % n) + 2; };
  auto iym = [&](std::size_t i) { return 4 * (i % n) + 3; };

  // Vertices are emitted per triangle (typical exporter layout) so each
  // face carries its own flat normal: outer and inner surfaces use the
  // in-plane normal perpendicular to the loop segment, the side walls
  // use +/-y.
  struct TriVert
  {
    double x;
    double y;
    double z;
    double nx;
    double ny;
    double nz;
  };
  std::vector<TriVert> verts;

  for (std::size_t i = 0; i < n; ++i)
  {
    const std::size_t j = (i + 1) % n;
    double dirX = loop[j].x - loop[i].x;
    double dirZ = loop[j].z - loop[i].z;
    double nX = dirZ;
    double nZ = -dirX;
    const double len = std::hypot(nX, nZ);
    if (len > 1e-9)
    {
      nX /= len;
      nZ /= len;
    }
    if (nX * (loop[i].x - cx) + nZ * (loop[i].z - cz) < 0.0)
    {
      nX = -nX;
      nZ = -nZ;
    }

    const std::size_t q[4][4] = {
      {oyp(i), oym(i), oym(j), oyp(j)},   // outer surface
      {iyp(i), iyp(j), iym(j), iym(i)},   // inner surface
      {oyp(i), oyp(j), iyp(j), iyp(i)},   // side wall +y
      {oym(i), iym(i), iym(j), oym(j)},   // side wall -y
    };
    const double normals[4][3] = {
      {nX, 0.0, nZ}, {-nX, 0.0, -nZ}, {0.0, 1.0, 0.0}, {0.0, -1.0, 0.0},
    };

    for (std::size_t f = 0; f < 4; ++f)
    {
      const std::size_t tri[2][3] = {
        {q[f][0], q[f][1], q[f][2]}, {q[f][0], q[f][2], q[f][3]},
      };
      for (const auto &t : tri)
      {
        for (std::size_t v = 0; v < 3; ++v)
        {
          const std::size_t id = t[v] % (4 * n);
          verts.push_back({vx[id], vy[id], vz[id],
                           normals[f][0], normals[f][1], normals[f][2]});
        }
      }
    }
  }

  const std::size_t triCount = verts.size() / 3;
  const std::size_t vertCount = verts.size();

  FILE *out = std::fopen(argv[1], "w");
  if (out == nullptr)
  {
    std::perror("belt_band_generator: fopen");
    return 1;
  }

  std::fprintf(out, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
  std::fprintf(out, "<!-- Generated continuous vl400 track band. Do not"
                    " edit. -->\n");
  std::fprintf(out, "<COLLADA xmlns=\"http://www.collada.org/2005/11/"
                    "COLLADASchema\" version=\"1.4.1\">\n");
  std::fprintf(out, "  <asset>\n");
  std::fprintf(out, "    <unit name=\"meter\" meter=\"1\"/>\n");
  std::fprintf(out, "    <up_axis>Z_UP</up_axis>\n");
  std::fprintf(out, "  </asset>\n");
  std::fprintf(out, "  <library_effects>\n");
  std::fprintf(out, "    <effect id=\"band-effect\">\n");
  std::fprintf(out, "      <profile_COMMON>\n");
  std::fprintf(out, "        <technique sid=\"common\">\n");
  std::fprintf(out, "          <lambert>\n");
  std::fprintf(out, "            <ambient>\n");
  std::fprintf(out, "              <color>0.05 0.05 0.05 1</color>\n");
  std::fprintf(out, "            </ambient>\n");
  std::fprintf(out, "            <diffuse>\n");
  std::fprintf(out, "              <color>0.05 0.05 0.05 1</color>\n");
  std::fprintf(out, "            </diffuse>\n");
  std::fprintf(out, "          </lambert>\n");
  std::fprintf(out, "        </technique>\n");
  std::fprintf(out, "      </profile_COMMON>\n");
  std::fprintf(out, "    </effect>\n");
  std::fprintf(out, "  </library_effects>\n");
  std::fprintf(out, "  <library_materials>\n");
  std::fprintf(out, "    <material id=\"band-material\" name=\"black\">\n");
  std::fprintf(out, "      <instance_effect url=\"#band-effect\"/>\n");
  std::fprintf(out, "    </material>\n");
  std::fprintf(out, "  </library_materials>\n");
  std::fprintf(out, "  <library_geometries>\n");
  std::fprintf(out, "    <geometry id=\"band-geometry\" name=\"band\">\n");
  std::fprintf(out, "      <mesh>\n");
  std::fprintf(out, "        <source id=\"band-positions\">\n");
  std::fprintf(out, "          <float_array id=\"band-positions-array\""
                    " count=\"%zu\">\n", 3 * vertCount);
  for (std::size_t i = 0; i < vertCount; ++i)
  {
    std::fprintf(out, "%.6f %.6f %.6f%s", verts[i].x, verts[i].y,
                 verts[i].z, ((i % 2) == 1) ? "\n" : " ");
  }
  std::fprintf(out, "          </float_array>\n");
  std::fprintf(out, "          <technique_common>\n");
  std::fprintf(out, "            <accessor source=\"#band-positions-array\""
                    " count=\"%zu\" stride=\"3\">\n", vertCount);
  std::fprintf(out, "              <param name=\"X\" type=\"float\"/>\n");
  std::fprintf(out, "              <param name=\"Y\" type=\"float\"/>\n");
  std::fprintf(out, "              <param name=\"Z\" type=\"float\"/>\n");
  std::fprintf(out, "            </accessor>\n");
  std::fprintf(out, "          </technique_common>\n");
  std::fprintf(out, "        </source>\n");
  std::fprintf(out, "        <source id=\"band-normals\">\n");
  std::fprintf(out, "          <float_array id=\"band-normals-array\""
                    " count=\"%zu\">\n", 3 * vertCount);
  for (std::size_t i = 0; i < vertCount; ++i)
  {
    std::fprintf(out, "%.6f %.6f %.6f%s", verts[i].nx, verts[i].ny,
                 verts[i].nz, ((i % 2) == 1) ? "\n" : " ");
  }
  std::fprintf(out, "          </float_array>\n");
  std::fprintf(out, "          <technique_common>\n");
  std::fprintf(out, "            <accessor source=\"#band-normals-array\""
                    " count=\"%zu\" stride=\"3\">\n", vertCount);
  std::fprintf(out, "              <param name=\"X\" type=\"float\"/>\n");
  std::fprintf(out, "              <param name=\"Y\" type=\"float\"/>\n");
  std::fprintf(out, "              <param name=\"Z\" type=\"float\"/>\n");
  std::fprintf(out, "            </accessor>\n");
  std::fprintf(out, "          </technique_common>\n");
  std::fprintf(out, "        </source>\n");
  std::fprintf(out, "        <vertices id=\"band-vertices\">\n");
  std::fprintf(out, "          <input semantic=\"POSITION\""
                    " source=\"#band-positions\"/>\n");
  std::fprintf(out, "        </vertices>\n");
  std::fprintf(out, "        <triangles material=\"band-material-sym\""
                    " count=\"%zu\">\n", triCount);
  std::fprintf(out, "          <input semantic=\"VERTEX\""
                    " source=\"#band-vertices\" offset=\"0\"/>\n");
  std::fprintf(out, "          <input semantic=\"NORMAL\""
                    " source=\"#band-normals\" offset=\"1\"/>\n");
  std::fprintf(out, "          <p>\n");
  for (std::size_t t = 0; t < triCount; ++t)
  {
    std::fprintf(out, "%zu %zu %zu %zu %zu %zu\n",
                 3 * t, 3 * t, 3 * t + 1, 3 * t + 1,
                 3 * t + 2, 3 * t + 2);
  }
  std::fprintf(out, "          </p>\n");
  std::fprintf(out, "        </triangles>\n");
  std::fprintf(out, "      </mesh>\n");
  std::fprintf(out, "    </geometry>\n");
  std::fprintf(out, "  </library_geometries>\n");
  std::fprintf(out, "  <library_visual_scenes>\n");
  std::fprintf(out, "    <visual_scene id=\"band-scene\">\n");
  std::fprintf(out, "      <node id=\"band-node\" name=\"band\">\n");
  std::fprintf(out, "        <instance_geometry url=\"#band-geometry\">\n");
  std::fprintf(out, "          <bind_material>\n");
  std::fprintf(out, "            <technique_common>\n");
  std::fprintf(out, "              <instance_material"
                    " symbol=\"band-material-sym\""
                    " target=\"#band-material\"/>\n");
  std::fprintf(out, "            </technique_common>\n");
  std::fprintf(out, "          </bind_material>\n");
  std::fprintf(out, "        </instance_geometry>\n");
  std::fprintf(out, "      </node>\n");
  std::fprintf(out, "    </visual_scene>\n");
  std::fprintf(out, "  </library_visual_scenes>\n");
  std::fprintf(out, "  <scene>\n");
  std::fprintf(out, "    <instance_visual_scene url=\"#band-scene\"/>\n");
  std::fprintf(out, "  </scene>\n");
  std::fprintf(out, "</COLLADA>\n");

  std::fclose(out);
  std::fprintf(stderr, "belt_band_generator: wrote %s (%zu loop points,"
                       " %zu triangles)\n", argv[1], n, triCount);
  return 0;
}
