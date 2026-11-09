// bench_step: what one 1 kHz step costs, through rc_twin and the Core
// directly. Driven by tools/bench.py, which writes the scenario file, repeats
// the runs, checks the machine is idle and summarises; run by hand it prints
// one JSON object per call.
//
//   bench_step <scenario.txt> <case> <mode> <steps> <reps>
//
// case: l0 | l1 | l2           level ground, no walls, SlipX's defaults
//       l2_road                l2 on the route's road ground (rc_twin_road_ground,
//                              which scans every centreline segment per wheel)
//       l2_unity               l2_road with the Unity twin's car (wheel inertia,
//                              ideal Ackermann, suspension, kinematic blend)
//       l2_unity_walls         l2_unity with the campus's kerbs and buildings
// mode: core                   slipx::VehicleModel::step, no C ABI (level only)
//       abi1                   rc_twin_step(n = 1) per step
//       abi10                  rc_twin_step(n = 10) per 100 Hz control tick,
//                              as sim_py's environments call it
//
// Every case drives the same closed loop: pure pursuit (0.6 m lookahead) and a
// proportional speed hold at 2 m/s along a campus route, the controller
// updated every 10 steps (100 Hz) from the state. At the route's end, or if the
// car leaves the road, it is put back at the start; those resets are counted
// and are inside the timed loop, as is the controller (a few ns per step).
//
// Not part of the default test run: built only with -DRC_TWIN_BENCH=ON.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rc_twin.h"
#include "slipx/state.hpp"
#include "slipx/vehicle_model.hpp"

namespace {

struct Scenario {
  std::vector<double> xyz;   // the driver's path: the route resampled, x y z triples
  std::vector<double> road;  // the road's centreline as the route gives it (sim_py's)
  double width = 1.4;
  double wheel_inertia = 0, ackermann = 0, wheel_rate = 0, damping = 0,
         travel = 0.015, v_kinematic = 0, footprint_margin = 0;
  std::vector<std::vector<double>> walls;  // x y pairs
  std::vector<int> closed;
};

bool read_scenario(const char* path, Scenario& sc) {
  std::ifstream in(path);
  if (!in) return false;
  std::string tag;
  while (in >> tag) {
    if (tag == "route") {
      size_t n;
      in >> n;
      sc.xyz.resize(3 * n);
      for (double& d : sc.xyz) in >> d;
    } else if (tag == "road") {
      size_t n;
      in >> n;
      sc.road.resize(3 * n);
      for (double& d : sc.road) in >> d;
    } else if (tag == "width") {
      in >> sc.width;
    } else if (tag == "unity") {
      in >> sc.wheel_inertia >> sc.ackermann >> sc.wheel_rate >> sc.damping >>
          sc.travel >> sc.v_kinematic >> sc.footprint_margin;
    } else if (tag == "walls") {
      size_t m;
      in >> m;
      for (size_t i = 0; i < m; ++i) {
        int closed;
        size_t n;
        in >> closed >> n;
        std::vector<double> xy(2 * n);
        for (double& d : xy) in >> d;
        sc.walls.push_back(std::move(xy));
        sc.closed.push_back(closed);
      }
    } else {
      return false;
    }
  }
  return in.eof() && sc.xyz.size() >= 27 && sc.road.size() >= 6;
}

// Pure pursuit along the route plus a speed hold. Tracks its own index.
struct Driver {
  const std::vector<double>* xyz;
  size_t n = 0, i = 0;
  double wheelbase = 0.32, steer_max = 0.4, lookahead = 0.6, v_target = 2.0;
  double steer = 0, accel = 0;
  bool at_end = false;

  double px(size_t k) const { return (*xyz)[3 * k]; }
  double py(size_t k) const { return (*xyz)[3 * k + 1]; }

  void update(double x, double y, double yaw, double vx) {
    // Nearest point, searching a little ahead of the last one.
    size_t best = i;
    double bd = 1e300;
    for (size_t k = i; k < n && k < i + 40; ++k) {
      const double d = std::hypot(px(k) - x, py(k) - y);
      if (d < bd) { bd = d; best = k; }
    }
    i = best;
    size_t j = i;
    while (j + 1 < n && std::hypot(px(j) - x, py(j) - y) < lookahead) ++j;
    at_end = (j + 1 >= n);
    const double dx = px(j) - x, dy = py(j) - y;
    const double lx = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double ly = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double ld2 = lx * lx + ly * ly;
    double s = std::atan2(2.0 * wheelbase * ly, ld2 > 1e-9 ? ld2 : 1e-9);
    if (s > steer_max) s = steer_max;
    if (s < -steer_max) s = -steer_max;
    steer = s;
    double a = 3.0 * (v_target - vx);
    accel = a > 4.0 ? 4.0 : (a < -4.0 ? -4.0 : a);
  }
};

// Start of the route: a little along it, heading along it, at rest.
void start_pose(const Scenario& sc, double& x, double& y, double& z, double& yaw) {
  x = sc.xyz[3 * 4];
  y = sc.xyz[3 * 4 + 1];
  z = sc.xyz[3 * 4 + 2];
  yaw = std::atan2(sc.xyz[3 * 8 + 1] - sc.xyz[3 * 4 + 1], sc.xyz[3 * 8] - sc.xyz[3 * 4]);
}

struct Result {
  double ns_per_step;
  unsigned long resets, contacts;
};

using clock_t_ = std::chrono::steady_clock;

Result run_core(const Scenario& sc, slipx::Tier tier, uint64_t steps) {
  auto model = slipx::VehicleModel::create(tier, slipx::VehicleParams{});
  Driver d{&sc.xyz};
  d.n = sc.xyz.size() / 3;
  slipx::VehicleState s{};
  auto reset = [&] {
    s = slipx::VehicleState{};
    double z;
    start_pose(sc, s.pos.x, s.pos.y, z, s.yaw);
    d.i = 0;
  };
  reset();
  Result r{0, 0, 0};
  const auto t0 = clock_t_::now();
  for (uint64_t k = 0; k < steps; ++k) {
    if (k % 10 == 0) {
      d.update(s.pos.x, s.pos.y, s.yaw, s.vel_body.x);
      if (d.at_end) { reset(); ++r.resets; d.update(s.pos.x, s.pos.y, s.yaw, 0); }
    }
    model->step(s, slipx::DriveInput{d.steer, d.accel}, 0.001);
  }
  const auto t1 = clock_t_::now();
  r.ns_per_step = double(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count()) /
                  double(steps);
  if (!std::isfinite(s.pos.x)) std::exit(3);
  return r;
}

Result run_abi(const Scenario& sc, const std::string& cs, uint32_t batch, uint64_t steps) {
  const int32_t tier = cs == "l0" ? RC_TWIN_TIER_L0_KINEMATIC
                       : cs == "l1" ? RC_TWIN_TIER_L1_BICYCLE
                                    : RC_TWIN_TIER_L2_DOUBLE_TRACK;
  const bool road_ground = cs.rfind("l2_", 0) == 0;
  const bool unity = cs.rfind("l2_unity", 0) == 0;
  const bool walls = cs == "l2_unity_walls";
  rc_twin_params p;
  rc_twin_params_default(&p);
  if (unity) {
    p.wheel_inertia = sc.wheel_inertia;
    p.ackermann = sc.ackermann;
    p.wheel_rate_front = p.wheel_rate_rear = sc.wheel_rate;
    p.damping_front = p.damping_rear = sc.damping;
    p.suspension_travel = sc.travel;
    p.v_kinematic = sc.v_kinematic;
  }
  const char* why = nullptr;
  rc_twin_vehicle* v = rc_twin_create(tier, RC_TWIN_INTEGRATOR_RK4, &p, &why);
  if (!v) { std::fprintf(stderr, "rc_twin_create: %s\n", why ? why : "?"); std::exit(2); }
  rc_twin_road* road = nullptr;
  if (road_ground) {
    road = rc_twin_road_create(sc.road.data(), nullptr, uint32_t(sc.road.size() / 3), sc.width, &why);
    if (!road) { std::fprintf(stderr, "rc_twin_road_create: %s\n", why ? why : "?"); std::exit(2); }
    rc_twin_set_ground_query(v, rc_twin_road_ground, road);
  }
  if (walls) {
    rc_twin_contact c;
    rc_twin_get_contact(v, &c);
    c.half_width += sc.footprint_margin;
    rc_twin_set_contact(v, &c);
    for (size_t w = 0; w < sc.walls.size(); ++w) {
      if (rc_twin_add_wall(v, sc.walls[w].data(), uint32_t(sc.walls[w].size() / 2),
                           sc.closed[w]) != RC_TWIN_OK) {
        std::fprintf(stderr, "rc_twin_add_wall failed\n");
        std::exit(2);
      }
    }
  }
  Driver d{&sc.xyz};
  d.n = sc.xyz.size() / 3;
  rc_twin_state st;
  auto reset = [&] {
    std::memset(&st, 0, sizeof st);
    start_pose(sc, st.x, st.y, st.z, st.yaw);
    st.soc = 0.8;
    st.pack_v = 11.1;
    rc_twin_reset(v, &st);
    d.i = 0;
  };
  reset();
  Result r{0, 0, 0};
  rc_twin_wall_report rep;
  const auto t0 = clock_t_::now();
  for (uint64_t k = 0; k < steps;) {
    rc_twin_get_state(v, &st);
    d.update(st.x, st.y, st.yaw, st.vx);
    if (d.at_end) { reset(); ++r.resets; d.update(st.x, st.y, st.yaw, 0); }
    for (uint32_t b = 0; b < 10 / batch && k < steps; ++b, k += batch) {
      const rc_twin_status rc = rc_twin_step(v, d.steer, d.accel, 0.001, batch);
      if (rc == RC_TWIN_E_GROUND) { reset(); ++r.resets; break; }
      if (rc != RC_TWIN_OK) { std::fprintf(stderr, "rc_twin_step: %d\n", rc); std::exit(2); }
      if (walls) {
        rc_twin_get_wall_report(v, &rep);
        r.contacts += rep.contacts;
      }
    }
  }
  const auto t1 = clock_t_::now();
  r.ns_per_step = double(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count()) /
                  double(steps);
  rc_twin_destroy(v);
  if (road) rc_twin_road_destroy(road);
  return r;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 6) {
    std::fprintf(stderr, "usage: bench_step <scenario.txt> <case> <mode> <steps> <reps>\n");
    return 1;
  }
  Scenario sc;
  if (!read_scenario(argv[1], sc)) {
    std::fprintf(stderr, "bench_step: cannot read scenario %s\n", argv[1]);
    return 1;
  }
  const std::string cs = argv[2], mode = argv[3];
  const uint64_t steps = std::strtoull(argv[4], nullptr, 10);
  const int reps = std::atoi(argv[5]);
  const bool known = cs == "l0" || cs == "l1" || cs == "l2" || cs == "l2_road" ||
                     cs == "l2_unity" || cs == "l2_unity_walls";
  if (!known || steps == 0 || reps <= 0 ||
      !(mode == "core" || mode == "abi1" || mode == "abi10") ||
      (mode == "core" && cs.size() > 2)) {
    std::fprintf(stderr, "bench_step: bad case/mode/steps/reps\n");
    return 1;
  }
  const slipx::Tier tier = cs == "l0" ? slipx::Tier::L0_Kinematic
                           : cs == "l1" ? slipx::Tier::L1_Bicycle
                                        : slipx::Tier::L2_DoubleTrack;
  auto once = [&](uint64_t n) {
    return mode == "core" ? run_core(sc, tier, n) : run_abi(sc, cs, mode == "abi1" ? 1 : 10, n);
  };
  once(steps / 10 > 0 ? steps / 10 : 1);  // warm-up, not reported
  std::printf("{\"case\": \"%s\", \"mode\": \"%s\", \"steps\": %llu, \"ns_per_step\": [", cs.c_str(),
              mode.c_str(), static_cast<unsigned long long>(steps));
  unsigned long resets = 0, contacts = 0;
  for (int i = 0; i < reps; ++i) {
    const Result r = once(steps);
    resets = r.resets;
    contacts = r.contacts;
    std::printf("%s%.3f", i ? ", " : "", r.ns_per_step);
  }
  std::printf("], \"resets_per_run\": %lu, \"wall_contacts_per_run\": %lu}\n", resets, contacts);
  return 0;
}
