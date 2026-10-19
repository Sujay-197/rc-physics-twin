// rc_twin road: ground from road data, for headless hosts (roads/
// road.schema.json, ADR 0005). Built from plain arrays so no JSON parser
// lives in the native layer; hosts parse the file. Answers the same ground
// query Unity's raycaster does.
//
// Geometry: a horizontal polyline centreline with elevation and banking at
// each point, linear between points. A query point is located by its nearest
// point on the centreline (arc length s, signed lateral offset d, left
// positive); the first and last segments extend straight beyond the ends.
// Height is the centreline's elevation, plus d tan(bank), plus any speed
// breaker. The normal follows from the gradient of that height. Off the
// road's width the query fails: not modelled.
#include "rc_twin.h"

#include <cmath>
#include <new>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Segment {
  double x0, y0, z0, tan_b0;  // start: position, tan(bank)
  double z1, tan_b1;          // end
  double tx, ty;              // unit direction, horizontal
  double len;                 // horizontal length              [m]
  double s0;                  // arc length at the start        [m]
};

struct Surface {
  double from_s, to_s, mu;
};

struct Breaker {
  double s, height, length;
};

}  // namespace

struct rc_twin_road {
  std::vector<Segment> seg;
  std::vector<Surface> surfaces;
  std::vector<Breaker> breakers;
  double width = 0.0;
  double length = 0.0;
};

namespace {

void set_reason(const char** reason, const char* why) {
  if (reason != nullptr) *reason = why;
}

// Nearest centreline point: segment index, along-segment distance u (not
// clamped on the end segments, which extend), and signed lateral offset.
void locate(const rc_twin_road& r, double x, double y, std::size_t* k_out,
            double* u_out, double* d_out) {
  double best = INFINITY;
  const std::size_t last = r.seg.size() - 1;
  for (std::size_t k = 0; k < r.seg.size(); ++k) {
    const Segment& g = r.seg[k];
    const double dx = x - g.x0;
    const double dy = y - g.y0;
    double u = dx * g.tx + dy * g.ty;
    if (u < 0.0 && k != 0) u = 0.0;
    if (u > g.len && k != last) u = g.len;
    const double px = g.x0 + u * g.tx;
    const double py = g.y0 + u * g.ty;
    const double ex = x - px;
    const double ey = y - py;
    const double dist2 = ex * ex + ey * ey;
    if (dist2 < best) {
      best = dist2;
      *k_out = k;
      *u_out = u;
      // Left of travel is positive: the cross product of direction and offset.
      const double cross = g.tx * ey - g.ty * ex;
      *d_out = (cross >= 0.0 ? 1.0 : -1.0) * std::sqrt(dist2);
    }
  }
}

bool finite(double v) { return std::isfinite(v); }

}  // namespace

extern "C" {

RC_TWIN_API rc_twin_road* rc_twin_road_create(const double* xyz,
                                              const double* bank,
                                              uint32_t n_points, double width,
                                              const char** reason) {
  set_reason(reason, nullptr);
  if (xyz == nullptr) {
    set_reason(reason, "xyz is NULL");
    return nullptr;
  }
  if (n_points < 2) {
    set_reason(reason, "a road needs at least two centreline points");
    return nullptr;
  }
  if (!(finite(width) && width > 0.0)) {
    set_reason(reason, "road width must be finite and positive [m]");
    return nullptr;
  }
  auto* r = new (std::nothrow) rc_twin_road;
  if (r == nullptr) {
    set_reason(reason, "rc_twin: out of memory");
    return nullptr;
  }
  r->width = width;
  double s = 0.0;
  for (uint32_t i = 0; i + 1 < n_points; ++i) {
    const double* a = xyz + 3 * i;
    const double* b = xyz + 3 * (i + 1);
    const double ba = bank != nullptr ? bank[i] : 0.0;
    const double bb = bank != nullptr ? bank[i + 1] : 0.0;
    for (double v : {a[0], a[1], a[2], b[0], b[1], b[2], ba, bb}) {
      if (!finite(v)) {
        set_reason(reason, "centreline values must be finite");
        delete r;
        return nullptr;
      }
    }
    if (std::fabs(ba) > 0.5 || std::fabs(bb) > 0.5) {
      set_reason(reason, "bank must be within +-0.5 rad");
      delete r;
      return nullptr;
    }
    const double len = std::hypot(b[0] - a[0], b[1] - a[1]);
    if (!(len > 1e-9)) {
      set_reason(reason, "consecutive centreline points must not coincide in x-y");
      delete r;
      return nullptr;
    }
    Segment g{};
    g.x0 = a[0];
    g.y0 = a[1];
    g.z0 = a[2];
    g.z1 = b[2];
    g.tan_b0 = std::tan(ba);
    g.tan_b1 = std::tan(bb);
    g.tx = (b[0] - a[0]) / len;
    g.ty = (b[1] - a[1]) / len;
    g.len = len;
    g.s0 = s;
    s += len;
    r->seg.push_back(g);
  }
  r->length = s;
  return r;
}

RC_TWIN_API void rc_twin_road_destroy(rc_twin_road* r) { delete r; }

RC_TWIN_API double rc_twin_road_length(const rc_twin_road* r) {
  return r != nullptr ? r->length : NAN;
}

RC_TWIN_API rc_twin_status rc_twin_road_add_surface(rc_twin_road* r,
                                                    double from_s, double to_s,
                                                    double mu_scale) {
  if (r == nullptr) return RC_TWIN_E_NULL;
  if (!(finite(from_s) && finite(to_s) && to_s > from_s && finite(mu_scale) &&
        mu_scale >= 0.0)) {
    return RC_TWIN_E_INVALID;
  }
  r->surfaces.push_back(Surface{from_s, to_s, mu_scale});
  return RC_TWIN_OK;
}

RC_TWIN_API rc_twin_status rc_twin_road_add_breaker(rc_twin_road* r, double s,
                                                    double height,
                                                    double length) {
  if (r == nullptr) return RC_TWIN_E_NULL;
  if (!(finite(s) && finite(height) && height > 0.0 && finite(length) &&
        length > 0.0)) {
    return RC_TWIN_E_INVALID;
  }
  r->breakers.push_back(Breaker{s, height, length});
  return RC_TWIN_OK;
}

RC_TWIN_API rc_twin_status rc_twin_road_locate(const rc_twin_road* r, double x,
                                               double y, double* s, double* d) {
  if (r == nullptr || s == nullptr || d == nullptr) return RC_TWIN_E_NULL;
  std::size_t k = 0;
  double u = 0.0;
  locate(*r, x, y, &k, &u, d);
  *s = r->seg[k].s0 + u;
  return RC_TWIN_OK;
}

RC_TWIN_API rc_twin_status rc_twin_road_point(const rc_twin_road* r, double s,
                                              double* x, double* y, double* z,
                                              double* heading) {
  if (r == nullptr || x == nullptr || y == nullptr || z == nullptr ||
      heading == nullptr) {
    return RC_TWIN_E_NULL;
  }
  if (!finite(s)) return RC_TWIN_E_INVALID;
  std::size_t k = 0;
  while (k + 1 < r->seg.size() && s > r->seg[k].s0 + r->seg[k].len) ++k;
  const Segment& g = r->seg[k];
  const double u = s - g.s0;  // may run past the ends: the end segments extend
  *x = g.x0 + u * g.tx;
  *y = g.y0 + u * g.ty;
  *z = g.z0 + (g.z1 - g.z0) * (u / g.len);
  *heading = std::atan2(g.ty, g.tx);
  return RC_TWIN_OK;
}

RC_TWIN_API int32_t rc_twin_road_ground(void* user, int32_t wheel, double x,
                                        double y, rc_twin_ground_sample* out) {
  (void)wheel;
  const auto* r = static_cast<const rc_twin_road*>(user);
  if (r == nullptr || out == nullptr) return 1;
  std::size_t k = 0;
  double u = 0.0;
  double d = 0.0;
  locate(*r, x, y, &k, &u, &d);
  if (std::fabs(d) > 0.5 * r->width) return 1;  // off the road: not modelled

  const Segment& g = r->seg[k];
  const double f = u / g.len;
  const double s = g.s0 + u;
  const double tan_b = g.tan_b0 + (g.tan_b1 - g.tan_b0) * f;

  double z = g.z0 + (g.z1 - g.z0) * f + d * tan_b;
  double dz_ds = (g.z1 - g.z0) / g.len + d * (g.tan_b1 - g.tan_b0) / g.len;
  const double dz_dd = tan_b;

  for (const Breaker& b : r->breakers) {
    const double ub = s - (b.s - 0.5 * b.length);
    if (ub >= 0.0 && ub <= b.length) {
      const double ph = kPi * ub / b.length;
      z += b.height * std::sin(ph) * std::sin(ph);
      dz_ds += b.height * kPi / b.length * std::sin(2.0 * ph);
    }
  }

  // Gradient in the world, along the travel direction t and its left l.
  const double gx = dz_ds * g.tx + dz_dd * (-g.ty);
  const double gy = dz_ds * g.ty + dz_dd * g.tx;
  const double inv = 1.0 / std::sqrt(gx * gx + gy * gy + 1.0);
  out->height = z;
  out->nx = -gx * inv;
  out->ny = -gy * inv;
  out->nz = inv;

  double mu = 1.0;
  for (const Surface& sf : r->surfaces) {
    if (s >= sf.from_s && s < sf.to_s) mu = sf.mu;
  }
  out->mu_scale = mu;
  return 0;
}

}  // extern "C"
