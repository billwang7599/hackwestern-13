// Physics of the virtual SM11: two DC TT gearmotors on an L298N-style EN+IN board,
// a sagging battery, skid-steer kinematics, walls, faults and the accelerometer.
// Plain C++17, no firmware dependencies, so it can be unit-tested on its own.
//
// Per wheel, every step (1 ms):
//   pins -> mode: coast (EN low), brake (EN high, IN1 == IN2), or drive (dir from IN1/IN2)
//   motor volts  Vm = duty * (V_batt - driver_drop_v)
//   static friction: a wheel at rest starts only if Vm >= v_start (+ scrub_v when spinning
//     in place); a rolling wheel stops if Vm < v_keep (+ scrub_v)
//   target speed = kv_mps_per_v * gain_side * (Vm - v_keep - scrub), first-order lag tau_s
//   brake: decays with brake_tau_s, coast: coast_tau_s
//   battery: V_batt = batt_v - I_batt * batt_r_ohm, I from (Vm - back-emf) / motor_r_ohm
// Ground: slip + seeded noise; v = (gl+gr)/2, w = (gr-gl)/track_eff_m + drift.
#pragma once
#include <cmath>
#include <map>
#include <random>
#include <string>
#include <vector>

struct ParamDef { const char *key; double dflt; const char *unit; const char *what; };

// Every tunable. The profile JSON overrides the defaults; {"set": {...}} on the control
// port changes them live. Keep in sync with profiles/sm11.json and bench/README.md.
static const ParamDef PARAM_DEFS[] = {
  {"batt_v",          7.4,   "V",      "battery open-circuit voltage (2x18650: 8.4 fresh, ~7.0 flat)"},
  {"batt_r_ohm",      0.30,  "ohm",    "battery + wiring internal resistance (sag under load)"},
  {"idle_a",          0.12,  "A",      "Uno + driver logic current"},
  {"driver_drop_v",   1.8,   "V",      "L298N bipolar drop at TT-motor currents"},
  {"kv_mps_per_v",    0.147, "m/s/V",  "ground speed per motor volt above v_keep (TT 1:48, 65 mm wheel)"},
  {"v_keep",          1.5,   "V",      "below this a ROLLING wheel stops (firmware DC_MIN_DUTY 0.30 = 1.68 V)"},
  {"v_start",         2.2,   "V",      "needed to START a wheel from rest (why the firmware kicks at 0.8)"},
  {"scrub_v",         0.15,  "V",      "extra loss when the sides run opposite (skid-steer spin)"},
  {"motor_r_ohm",     6.5,   "ohm",    "TT motor winding resistance (current, sag, stall)"},
  {"tau_s",           0.08,  "s",      "driven speed time constant"},
  {"brake_tau_s",     0.03,  "s",      "speed decay when braked (EN high, IN1 == IN2)"},
  {"coast_tau_s",     0.25,  "s",      "speed decay when coasting (EN low / reset / unplugged power)"},
  {"gain_l",          1.0,   "",       "left side speed gain (drift source: 490 vs 980 Hz PWM)"},
  {"gain_r",          1.0,   "",       "right side speed gain"},
  {"slip",            0.03,  "",       "fraction of wheel speed lost to the floor"},
  {"drift_deg_per_m", 0.0,   "deg/m",  "extra heading drift per metre driven, + = curves right"},
  {"noise_mps",       0.002, "m/s",    "per-step ground speed noise (seeded)"},
  {"track_eff_m",     0.20,  "m",      "effective skid-steer track: spin rate = (gr-gl)/track"},
  {"car_r_m",         0.12,  "m",      "collision radius around the car centre"},
  {"bump_g_per_mps",  16.0,  "g/(m/s)","accel spike per m/s of impact speed (bump threshold 1.2 g)"},
  {"imu_noise_g",     0.01,  "g",      "accelerometer noise"},
  {"uno_resets_below_v", 6.0, "V",     "brownout: Uno resets if the battery falls below this (0 = never)"},
  {"boot_ms",         1600,  "ms",     "reset to READY (Optiboot wait + setup)"},
  {"imu",             1,     "",       "accelerometer: 0 none, 1 MPU6050, 2 ADXL345"},
};

struct Wall { double x1, y1, x2, y2; };

struct DriveIn {      // one motor channel, read from the firmware's pins
  int mode = 0;       // 0 coast, 1 brake, 2 drive
  double duty = 0;    // 0..1
  int dir = 0;        // +1 forward, -1 back
};

struct Spike { double gx, gy, gz; double until_s; };

struct Car {
  std::map<std::string, double> p;
  // state
  double t = 0;                       // sim seconds
  double x = 0, y = 0, th = 0;        // world frame (bench/README.md)
  double s[2] = {0, 0};               // wheel surface speeds m/s (+ = forward)
  double v = 0, w = 0, v_prev = 0, ax_lp = 0;  // body
  double vb = 7.4, ib = 0;            // battery terminal V, current A
  double pvx = 0, pvy = 0;            // external push velocity, world frame, decays
  bool contact = false;
  double roll_deg = 0;                // physical tilt (faults)
  bool f_tip = false, f_lift[2] = {false, false};
  double brown_v = 0, brown_until = -1;
  double imu[3] = {0, 0, 1};          // specific force, body frame, g
  double odo_m = 0;                   // distance driven (ground), for tests
  std::vector<Wall> walls;
  std::vector<Spike> spikes;
  std::mt19937 rng;
  std::normal_distribution<double> nd{0.0, 1.0};
  DriveIn last_in[2];

  explicit Car(unsigned seed = 1) : rng(seed) {
    for (auto &d : PARAM_DEFS) p[d.key] = d.dflt;
    vb = p["batt_v"];
  }
  static bool known(const std::string &k) {
    for (auto &d : PARAM_DEFS) if (k == d.key) return true;
    return false;
  }
  double P(const char *k) const { auto it = p.find(k); return it == p.end() ? 0 : it->second; }
  double n01() { return nd(rng); }

  bool browned() const { return brown_until > t; }
  double supply_v() const { return browned() ? brown_v : P("batt_v"); }
  bool tipped() const { return f_tip; }

  void addSpike(double gx, double gy, double gz, double dur_s) { spikes.push_back({gx, gy, gz, t + dur_s}); }

  void step(double dt, const DriveIn in[2]) {
    t += dt;
    last_in[0] = in[0]; last_in[1] = in[1];
    const double drop = P("driver_drop_v"), kv = P("kv_mps_per_v");
    bool spin = in[0].mode == 2 && in[1].mode == 2 && in[0].dir != in[1].dir;
    double i_motor_total = 0;
    for (int k = 0; k < 2; k++) {
      const DriveIn &d = in[k];
      bool lifted = f_lift[k] || f_tip;
      double gain = k == 0 ? P("gain_l") : P("gain_r");
      double keep = lifted ? 0.8 : P("v_keep") + (spin ? P("scrub_v") : 0);
      double start = lifted ? 0.9 : P("v_start") + (spin ? P("scrub_v") : 0);
      double target = 0, tau = P("coast_tau_s");
      double vm = 0;
      if (d.mode == 2) {
        double supply = vb - drop;
        if (supply < 0) supply = 0;
        vm = d.duty * supply;
        bool atRest = std::fabs(s[k]) < 1e-4;
        double need = atRest ? start : keep;
        if (vm >= need) target = d.dir * kv * gain * (vm - keep) * (lifted ? 1.3 : 1.0);
        tau = vm >= need ? P("tau_s") : P("brake_tau_s") * 2;   // stalled: friction stops it
        // current: (Vm - back emf) / R, back emf from the current speed
        double emf = std::fabs(s[k]) / (kv * (gain > 0.05 ? gain : 0.05));
        double i = (vm - emf) / P("motor_r_ohm");
        if (i < 0) i = 0;
        i_motor_total += d.duty * i;   // battery side of a PWM'd motor
      } else if (d.mode == 1) {
        tau = P("brake_tau_s");
      }
      if (tau < 1e-3) tau = 1e-3;
      s[k] += (target - s[k]) * (dt / tau > 1 ? 1 : dt / tau);
      if (target == 0 && std::fabs(s[k]) < 2e-3) s[k] = 0;
    }
    ib = i_motor_total + P("idle_a");
    vb = supply_v() - ib * P("batt_r_ohm");
    if (vb < 0) vb = 0;

    // ground
    double g[2];
    for (int k = 0; k < 2; k++) {
      bool lifted = f_lift[k] || f_tip;
      g[k] = lifted ? 0 : s[k] * (1 - P("slip"));
      if (g[k] != 0) g[k] += P("noise_mps") * n01();
    }
    double vv = (g[0] + g[1]) / 2;
    double ww = (g[1] - g[0]) / (P("track_eff_m") > 0.02 ? P("track_eff_m") : 0.02);
    ww -= P("drift_deg_per_m") * M_PI / 180.0 * std::fabs(vv);
    if (f_tip) { vv = 0; ww = 0; }

    // push decays
    double pk = std::exp(-dt / 0.15);
    pvx *= pk; pvy *= pk;

    double ox = x, oy = y;
    th += ww * dt;
    th = std::atan2(std::sin(th), std::cos(th));
    x += (vv * std::cos(th) + pvx) * dt;
    y += (vv * std::sin(th) + pvy) * dt;

    // walls: push out, stall the wheels when driving into one, spike on impact
    bool was = contact;
    contact = false;
    double R = P("car_r_m");
    for (auto &wl : walls) {
      double ex = wl.x2 - wl.x1, ey = wl.y2 - wl.y1;
      double L2 = ex * ex + ey * ey;
      double u = L2 > 0 ? ((x - wl.x1) * ex + (y - wl.y1) * ey) / L2 : 0;
      u = u < 0 ? 0 : u > 1 ? 1 : u;
      double cx = wl.x1 + u * ex, cy = wl.y1 + u * ey;
      double dx = x - cx, dy = y - cy;
      double d = std::sqrt(dx * dx + dy * dy);
      if (d >= R) continue;
      contact = true;
      double nx, ny;
      if (d > 1e-9) { nx = dx / d; ny = dy / d; }
      else { nx = -std::cos(th); ny = -std::sin(th); }
      x = cx + nx * R; y = cy + ny * R;
      double hx = std::cos(th), hy = std::sin(th);
      double vn = (vv * hx + pvx) * nx + (vv * hy + pvy) * ny;   // < 0 = into the wall
      double hn = hx * nx + hy * ny;                             // -1 = facing it head-on
      if (vv * hn < 0) {                                         // driving into it: stall
        double f = std::fabs(hn);
        s[0] *= 1 - f; s[1] *= 1 - f;
      }
      double pn = pvx * nx + pvy * ny;
      if (pn < 0) { pvx -= pn * nx; pvy -= pn * ny; }
      if (!was && vn < -0.02) {
        double gmag = P("bump_g_per_mps") * -vn;
        // decel along the wall normal, in the body frame
        double bx = nx * hx + ny * hy, by = -nx * hy + ny * hx;
        addSpike(gmag * bx, gmag * by, 0, 0.02);
      }
    }
    double dxm = x - ox, dym = y - oy;
    double moved = std::sqrt(dxm * dxm + dym * dym);
    odo_m += moved;
    // actual body speed (after walls), signed by heading
    double hx = std::cos(th), hy = std::sin(th);
    v = dt > 0 ? (dxm * hx + dym * hy) / dt : 0;
    w = ww;

    // accelerometer (specific force, body frame: x fwd, y left, z up; MPU flat)
    // forward accel from the wheels' (noise-free) speed, low-passed 10 ms; wall hits
    // come in as explicit spikes, so a stall doesn't read as a 10 g jolt here
    double vs = f_tip ? 0 : (s[0] + s[1]) / 2;
    double araw = dt > 0 ? (vs - v_prev) / dt / 9.81 : 0;
    v_prev = vs;
    if (araw > 3) araw = 3; else if (araw < -3) araw = -3;
    ax_lp += (araw - ax_lp) * (dt / 0.01 > 1 ? 1 : dt / 0.01);
    double ax = ax_lp;
    double ay = v * w / 9.81;
    roll_deg = f_tip ? 75 : f_lift[0] ? 12 : f_lift[1] ? -12 : 0;
    double r = roll_deg * M_PI / 180;
    double gx = 0, gy = std::sin(r), gz = std::cos(r);
    double sx = 0, sy = 0, sz = 0;
    std::vector<Spike> keepS;
    for (auto &sp : spikes) if (sp.until_s > t) { sx += sp.gx; sy += sp.gy; sz += sp.gz; keepS.push_back(sp); }
    spikes.swap(keepS);
    double nz = P("imu_noise_g");
    imu[0] = ax + gx + sx + nz * n01();
    imu[1] = ay + gy + sy + nz * n01();
    imu[2] = gz + sz + nz * n01();
  }
};
