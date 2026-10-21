/*
 * rc_twin: C ABI around the SlipX vehicle Core (ADR 0002).
 *
 * Plain C, fixed-layout structs of doubles and int32s, so Unity can P/Invoke
 * it and a headless harness can link it unchanged. The one callback is the
 * optional ground query (rc_twin_set_ground_query).
 *
 * Conventions are SlipX's (core/src/core/slipx_core/include/slipx/
 * conventions.hpp): ISO 8855, SI units. World x/y in the ground plane, z up;
 * body x forward, y LEFT, z up; yaw positive counter-clockwise from above;
 * steer positive LEFT. Per-wheel arrays are ordered FL, FR, RL, RR.
 *
 * Every function that takes a handle or out-pointer returns an rc_twin_status;
 * none throws or aborts. Strings returned through `reason` are static and
 * never need freeing.
 *
 * Bump RC_TWIN_ABI_VERSION on any change to a struct layout or signature,
 * and on any added function, so a host built against a newer header fails
 * loudly on an older DLL.
 *
 * v2: ground query (rc_twin_ground_sample, rc_twin_set_ground_query,
 *     RC_TWIN_E_GROUND).
 * v3: rc_twin_params.wheel_inertia (ADR 0006).
 * v4: rc_twin_params.ackermann (ADR 0007).
 * v5: rc_twin_params suspension fields (ADR 0008); with suspension on, the
 *     state's z, pitch, roll, vz, pitch_rate and roll_rate are the body's.
 * v6: roads (rc_twin_road_*), ground from road data for headless hosts.
 * v7: walls (rc_twin_add_wall, rc_twin_contact, rc_twin_wall_report;
 *     ADR 0009).
 */
#ifndef RC_TWIN_H
#define RC_TWIN_H

#include <stdint.h>

#if defined(_WIN32)
#  if defined(RC_TWIN_BUILDING)
#    define RC_TWIN_API __declspec(dllexport)
#  else
#    define RC_TWIN_API __declspec(dllimport)
#  endif
#else
#  define RC_TWIN_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define RC_TWIN_ABI_VERSION 7u

typedef int32_t rc_twin_status;
enum {
  RC_TWIN_OK = 0,
  RC_TWIN_E_NULL = -1,     /* a required pointer was NULL */
  RC_TWIN_E_INVALID = -2,  /* bad argument or physically impossible params */
  RC_TWIN_E_INTERNAL = -3, /* the Core threw; the vehicle state is unchanged */
  RC_TWIN_E_GROUND = -4    /* the ground query failed, returned an invalid
                              sample, or described ground this tier cannot
                              represent; the vehicle state is unchanged */
};

/* Fidelity tier, SlipX slipx::Tier. L3 is not implemented upstream yet. */
enum {
  RC_TWIN_TIER_L0_KINEMATIC = 0,
  RC_TWIN_TIER_L1_BICYCLE = 1,
  RC_TWIN_TIER_L2_DOUBLE_TRACK = 2,
  RC_TWIN_TIER_L3_EXTENDED = 3
};

enum { RC_TWIN_INTEGRATOR_RK4 = 0, RC_TWIN_INTEGRATOR_SEMI_IMPLICIT_EULER = 1 };
enum { RC_TWIN_PROVISIONAL = 0, RC_TWIN_IDENTIFIED = 1, RC_TWIN_MEASURED = 2 };
enum { RC_TWIN_RWD = 0, RC_TWIN_FWD = 1, RC_TWIN_AWD = 2 };
enum { RC_TWIN_DIFF_OPEN = 0, RC_TWIN_DIFF_SPOOL = 1, RC_TWIN_DIFF_LSD = 2 };

/* slipx::VehicleState, flattened. Field meanings and units as upstream. */
typedef struct rc_twin_state {
  double x, y, z;                         /* CoG world position        [m]   */
  double yaw, pitch, roll;                /*                           [rad] */
  double vx, vy, vz;                      /* body-frame velocity       [m/s] */
  double roll_rate, pitch_rate, yaw_rate; /* body rates               [rad/s]*/
  double omega_w[4];                      /* wheel speeds, L2+        [rad/s]*/
  double steer;                           /* achieved road wheel angle [rad] */
  double steer_rate;                      /* L2+                      [rad/s]*/
  double soc;                             /* state of charge, L2+      [-]   */
  double pack_v;                          /* pack terminal voltage     [V]   */
  double fz[4];                           /* vertical tyre loads, L2+  [N]   */
  double alpha_lag[4];                    /* lagged slip angles, L2+   [rad] */
} rc_twin_state;

/* slipx::TyreCoefficients. */
typedef struct rc_twin_tyre {
  double mu_y0, mu_x0, k_mu, relax_length, shape_c, curvature_e;
} rc_twin_tyre;

/* slipx::VehicleParams, flattened. Fill with rc_twin_params_default() first,
 * then change what you need: struct_size is how the library checks that the
 * caller was built against the same layout. */
typedef struct rc_twin_params {
  uint32_t struct_size;  /* = sizeof(rc_twin_params); set by _default() */
  int32_t provenance;    /* RC_TWIN_PROVISIONAL etc. */

  double mass, izz, ixx, iyy;
  double lf, lr, track_front, track_rear, h_cog, wheel_radius;

  double c_alpha_f, c_alpha_r, mu_clip;
  rc_twin_tyre tyre_front, tyre_rear;
  double c_kappa;

  double accel_max, decel_max, v_max;
  int32_t layout;        /* RC_TWIN_RWD etc. */
  int32_t differential;  /* RC_TWIN_DIFF_OPEN etc. */
  double lsd_preload;

  double torque_stall, omega_free, torque_per_amp, drive_efficiency;
  double current_max, regen_current_max;

  double pack_nominal_v, pack_v_full, pack_v_empty, pack_capacity_ah,
      pack_internal_resistance;

  double steer_max, steer_rate_max, steer_bandwidth, steer_damping;

  double drag_coeff, roll_resist, v_eps;

  /* Per wheel, including reflected drivetrain inertia [kg m^2]. 0 (the
   * default) keeps quasi-static wheels; > 0 makes wheel speeds integrated
   * state at L2, so wheelspin and lock-up happen (ADR 0006). */
  double wheel_inertia;

  /* Fraction of ideal Ackermann at L2: 0 parallel steer (the default), 1
   * ideal, negative anti-Ackermann; in [-1, 1] (ADR 0007). */
  double ackermann;

  /* Suspension at L2 (ADR 0008). On when both wheel rates are positive; all
   * 0 (the default) keeps quasi-static load transfer. Rates and damping are
   * per wheel, measured at the wheel; travel is compression from static to
   * the bump stop. With it on, z is the world height of the point h_cog below
   * the CoG, pitch is nose up and roll right side down, absolute. */
  double wheel_rate_front, wheel_rate_rear;   /* [N/m]   */
  double damping_front, damping_rear;         /* [N s/m] */
  double suspension_travel;                   /* [m]     */
} rc_twin_params;

typedef struct rc_twin_vehicle rc_twin_vehicle;

/* The ground under one wheel (ADR 0005). World frame, ISO 8855. */
typedef struct rc_twin_ground_sample {
  double height;     /* ground z under the wheel                        [m] */
  double nx, ny, nz; /* outward normal, any length, nz > 0                  */
  double mu_scale;   /* multiplies the tyre's peak friction, >= 0       [-] */
} rc_twin_ground_sample;

/* Ground query, called once per wheel (0 FL, 1 FR, 2 RL, 3 RR) before every
 * Core step, at that wheel's centre projected to the ground (world x, y) at
 * the start of the step. *out arrives filled with level ground at height 0
 * and mu_scale 1. Return 0 on success; anything else aborts the call to
 * rc_twin_step with RC_TWIN_E_GROUND. Called on the thread that called
 * rc_twin_step, never after rc_twin_step returns. */
typedef int32_t (*rc_twin_ground_fn)(void* user, int32_t wheel, double x,
                                     double y, rc_twin_ground_sample* out);

RC_TWIN_API uint32_t rc_twin_abi_version(void);
/* SlipX version string of the vendored Core, e.g. "0.2.0". */
RC_TWIN_API const char* rc_twin_core_version(void);

/* SlipX's provisional defaults (a generic 1/10 electric car). */
RC_TWIN_API rc_twin_status rc_twin_params_default(rc_twin_params* out);

/* Returns NULL with *reason set on failure (reason may be NULL). The vehicle
 * starts at rest at the origin, as after rc_twin_reset(v, NULL). */
RC_TWIN_API rc_twin_vehicle* rc_twin_create(int32_t tier, int32_t integrator,
                                            const rc_twin_params* params,
                                            const char** reason);
RC_TWIN_API void rc_twin_destroy(rc_twin_vehicle* v);

/* Rebuilds the model with new params; the current state is kept. On failure
 * the old params stay in force. */
RC_TWIN_API rc_twin_status rc_twin_set_params(rc_twin_vehicle* v,
                                              const rc_twin_params* params,
                                              const char** reason);
RC_TWIN_API rc_twin_status rc_twin_get_params(const rc_twin_vehicle* v,
                                              rc_twin_params* out);

/* initial == NULL resets to rest at the origin (slipx::VehicleState{}). */
RC_TWIN_API rc_twin_status rc_twin_reset(rc_twin_vehicle* v,
                                         const rc_twin_state* initial);
RC_TWIN_API rc_twin_status rc_twin_get_state(const rc_twin_vehicle* v,
                                             rc_twin_state* out);

/* Install a ground query; fn == NULL restores level ground at height 0 (the
 * default). Only L2 represents grade and grip: at L0/L1 a query that returns
 * anything but level, unscaled ground makes rc_twin_step fail with
 * RC_TWIN_E_GROUND. */
RC_TWIN_API rc_twin_status rc_twin_set_ground_query(rc_twin_vehicle* v,
                                                    rc_twin_ground_fn fn,
                                                    void* user);

/* Advance n fixed steps of dt seconds with a held command. steer_cmd is the
 * commanded road wheel angle [rad, + left]; accel_cmd the longitudinal
 * acceleration demand [m/s^2, + forward]. dt must be finite and > 0, n >= 1.
 * Exactly equivalent to n calls with n = 1. On any failure no step is kept:
 * the state is as it was before the call.
 *
 * Reverse (ADR 0010): a negative demand brakes; held with the car stopped it
 * puts the ESC in reverse, and the car drives backwards until it next rolls
 * forward. rc_twin_reset leaves reverse. With wheel_inertia 0 the Core's
 * negative demand reverses from rest anyway, and nothing changes. */
RC_TWIN_API rc_twin_status rc_twin_step(rc_twin_vehicle* v, double steer_cmd,
                                        double accel_cmd, double dt,
                                        uint32_t n);

/* ------------------------------------------------------------------ walls
 * Immovable plan-view segments the car's footprint cannot cross (ADR 0009,
 * SlipX ADR-0055). After every Core step the footprint is checked against
 * each wall segment in the order added; a touch applies one impulse with
 * restitution and Coulomb friction to the body velocity and yaw rate, and
 * moves the CoG out of the penetration toward the side its centre is on.
 * Walls survive rc_twin_reset and rc_twin_set_params. With no walls, steps
 * are bit-identical to a vehicle that never had any. */

/* Contact constants and the footprint. The constants are SlipX's
 * ContactParams, provisional by nature; the footprint is the car's plan
 * outline, half_length forward and back and half_width to each side of a
 * centre centre_offset ahead of the CoG [m]. */
typedef struct rc_twin_contact {
  double restitution;           /* 0 dead stop .. 1 elastic          [-]   */
  double friction;              /* tangential / normal impulse cap   [-]   */
  double restitution_min_speed; /* no bounce below this closing speed [m/s] */
  double half_length, half_width, centre_offset; /*                  [m]   */
} rc_twin_contact;

/* What the last rc_twin_step call's wall contacts did, over all its steps. */
typedef struct rc_twin_wall_report {
  uint32_t contacts;    /* segment contacts resolved                       */
  uint32_t reserved;
  double impulse;       /* total normal impulse                     [N s] */
  double max_approach;  /* highest closing speed at a contact       [m/s] */
} rc_twin_wall_report;

/* SlipX's constants and the footprint from the params' geometry: the
 * wheelbase plus a wheel radius at each end, the wider track across,
 * centred between the axles. */
RC_TWIN_API rc_twin_status rc_twin_contact_default(const rc_twin_params* p,
                                                   rc_twin_contact* out);
/* Every vehicle starts with rc_twin_contact_default of its params; a later
 * rc_twin_set_params keeps whatever contact is set. Values must be finite,
 * restitution in [0, 1], friction and the speed >= 0, half extents > 0. */
RC_TWIN_API rc_twin_status rc_twin_set_contact(rc_twin_vehicle* v,
                                               const rc_twin_contact* c);
RC_TWIN_API rc_twin_status rc_twin_get_contact(const rc_twin_vehicle* v,
                                               rc_twin_contact* out);
/* A wall along n_points (x, y) pairs [m], n_points >= 2; closed != 0 also
 * joins the last point to the first (n_points >= 3). Zero-length segments
 * are dropped. All or nothing: on failure no segment is added. */
RC_TWIN_API rc_twin_status rc_twin_add_wall(rc_twin_vehicle* v,
                                            const double* xy, uint32_t n_points,
                                            int32_t closed);
RC_TWIN_API rc_twin_status rc_twin_clear_walls(rc_twin_vehicle* v);
/* Number of wall segments; 0 for NULL. */
RC_TWIN_API uint32_t rc_twin_wall_count(const rc_twin_vehicle* v);
RC_TWIN_API rc_twin_status rc_twin_get_wall_report(const rc_twin_vehicle* v,
                                                   rc_twin_wall_report* out);

/* ------------------------------------------------------------------ roads
 * Ground from road data (roads/road.schema.json), for hosts without a scene:
 * the headless harness, CI, the edge profiler. Built from arrays; hosts parse
 * the file. Pass rc_twin_road_ground with the road as user to
 * rc_twin_set_ground_query. Off the road's width the query fails, so
 * rc_twin_step returns RC_TWIN_E_GROUND: leaving the road is not modelled. */
typedef struct rc_twin_road rc_twin_road;

/* n_points centreline points as x, y, z triples [m]; bank [rad] per point,
 * positive when the road rises to the left of travel (NULL: level). */
RC_TWIN_API rc_twin_road* rc_twin_road_create(const double* xyz,
                                              const double* bank,
                                              uint32_t n_points, double width,
                                              const char** reason);
RC_TWIN_API void rc_twin_road_destroy(rc_twin_road* r);
/* Horizontal centreline length [m]; NaN for NULL. */
RC_TWIN_API double rc_twin_road_length(const rc_twin_road* r);
/* A stretch [from_s, to_s) with this grip; later stretches win. */
RC_TWIN_API rc_twin_status rc_twin_road_add_surface(rc_twin_road* r,
                                                    double from_s, double to_s,
                                                    double mu_scale);
/* A round hump, height sin^2 over its length, centred at arc length s. */
RC_TWIN_API rc_twin_status rc_twin_road_add_breaker(rc_twin_road* r, double s,
                                                    double height,
                                                    double length);
/* Nearest centreline point to (x, y): arc length s and signed lateral offset
 * d (left positive). */
RC_TWIN_API rc_twin_status rc_twin_road_locate(const rc_twin_road* r, double x,
                                               double y, double* s, double* d);
/* The centreline at arc length s: position and heading [rad]. */
RC_TWIN_API rc_twin_status rc_twin_road_point(const rc_twin_road* r, double s,
                                              double* x, double* y, double* z,
                                              double* heading);
/* An rc_twin_ground_fn; user is the rc_twin_road. */
RC_TWIN_API int32_t rc_twin_road_ground(void* road, int32_t wheel, double x,
                                        double y, rc_twin_ground_sample* out);

#ifdef __cplusplus
}
#endif

#endif /* RC_TWIN_H */
