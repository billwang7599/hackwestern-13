// Physics of the virtual SM11: DC TT gearmotors on an L298N-style EN+IN board (1 or 2
// motors in parallel per channel = 2WD or 4WD), a sagging battery, skid-steer kinematics
// with tyre scrub, a rectangular body against walls, faults and the accelerometer.
// Plain C++17, no firmware dependencies, so it can be unit-tested on its own.
//
// Per side (= one driver channel, motors_per_side motors in parallel), every 1 ms step:
//   pins -> mode: coast (EN low), brake (EN high, IN1 == IN2), or drive (dir from IN1/IN2)
//   driver drop  = driver_drop_v + driver_r_ohm * I_channel (L298N: grows with current)
//   motor volts  Vm = duty * (V_batt - drop), clamped so I_channel <= ch_limit_a
//   static friction: a wheel at rest starts only if Vm >= v_start (+ scrub), a rolling wheel
//     stops if Vm < v_keep (+ scrub). v_start / v_keep are PER MOTOR (gearbox + its share of
//     rolling load). scrub = scrub_v (spin only, legacy) + f * V_geo, where
//       V_geo = scrub_mu * mass_kg * g * wheelbase_m / (2 * track_m)          [N per side]
//               / (motors_per_side * motor_n_per_v)                            [-> V per motor]
//     is the extra drive force a 4-wheel skid-steer needs to drag its tyres sideways in a
//     spin (f = 1 for a spin, ~0 straight). 2 motors per side halve it (twice the torque).
//     wheelbase_m = 0 (2WD on a caster) means no geometric scrub.
//   target speed = kv_mps_per_v * gain_side * (Vm - keep), first-order lag tau_s / motors_per_side
//   brake: decays with brake_tau_s, coast: coast_tau_s
//   current per motor (Vm - back-emf) / motor_r_ohm; channel = motors_per_side x that
//   battery: V_batt = batt_v - I_batt * batt_r_ohm, I_batt = sum(duty * I_channel) + idle_a
//   batt_v_min = the PWM on-phase sag (both channels on at once); below uno_resets_below_v
//     for brown_hold_ms -> the Uno resets (sag_low(); vuno holds the firmware in reset)
// Ground: slip + seeded noise; v = (gl+gr)/2, w = (gr-gl)/track_eff + drift, where
//   track_eff = track_eff_m if > 0, else track_m * (1 + (wheelbase_m/track_m)^2) (a rough
//   skid-steer ICR-spread factor; the literature measures 1.4-2x on 4-wheel skid-steers).
// Body: a circle of car_r_m, or a length_m x width_m rectangle when length_m > 0 (corners
//   sweep the half-diagonal in a spin and can clip a box the centre never gets near).
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
  {"track_eff_m",     0.20,  "m",      "effective skid-steer track: spin rate = (gr-gl)/track; 0 = derive from track_m, wheelbase_m"},
  {"car_r_m",         0.12,  "m",      "collision radius around the car centre (when length_m = 0)"},
  // ---- 4WD / skid-steer / body (defaults keep a 2WD circle car exactly as before)
  {"motors_per_side", 1,     "",       "TT motors in parallel on each driver channel: 1 = 2WD, 2 = 4WD"},
  {"driver_r_ohm",    0.0,   "ohm",    "extra driver drop per channel amp (L298N VCEsat grows with current)"},
  {"ch_limit_a",      0.0,   "A",      "per-channel current the driver can deliver (L298N ~2, L293D ~0.6; 0 = none)"},
  {"track_m",         0.0,   "m",      "geometric track, wheel centre to wheel centre"},
  {"wheelbase_m",     0.0,   "m",      "front to rear axle (0 = 2WD on a caster: no tyre scrub in a spin)"},
  {"mass_kg",         0.5,   "kg",     "whole car incl. payload (tyre scrub load)"},
  {"scrub_mu",        0.0,   "",       "sideways tyre friction while scrubbing in a spin (0 = off)"},
  {"motor_n_per_v",   0.4,   "N/V",    "ground force per motor per volt above v_keep (TT 1:48 stall torque / wheel radius)"},
  {"length_m",        0.0,   "m",      "body length for collisions (0 = use the car_r_m circle)"},
  {"width_m",         0.0,   "m",      "body width incl. wheels"},
  {"brown_hold_ms",   5,     "ms",     "sag below uno_resets_below_v this long resets the Uno"},
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

// Oriented box vs segment, separating-axis test. Returns penetration depth (0 = apart)
// and the unit normal pushing the box out of the segment.
inline double box_seg_pen(double cx, double cy, double th, double a, double b, const Wall &w,
                          double &nx, double &ny) {
  const double ux = std::cos(th), uy = std::sin(th), vx = -uy, vy = ux;
  double ex = w.x2 - w.x1, ey = w.y2 - w.y1, el = std::sqrt(ex * ex + ey * ey);
  double axes[3][2] = {{ux, uy}, {vx, vy}, {el > 1e-12 ? -ey / el : vx, el > 1e-12 ? ex / el : vy}};
  double best = 1e30;
  for (auto &k : axes) {
    double r = a * std::fabs(ux * k[0] + uy * k[1]) + b * std::fabs(vx * k[0] + vy * k[1]);
    double c = cx * k[0] + cy * k[1];
    double p1 = w.x1 * k[0] + w.y1 * k[1], p2 = w.x2 * k[0] + w.y2 * k[1];
    double lo = p1 < p2 ? p1 : p2, hi = p1 < p2 ? p2 : p1;
    double ov = std::fmin(c + r - lo, hi - (c - r));
    if (ov <= 0) return 0;
    if (ov < best) {
      best = ov;
      double sgn = c - (lo + hi) / 2 >= 0 ? 1 : -1;
      nx = k[0] * sgn; ny = k[1] * sgn;
    }
  }
  return best;
}

struct Car {
  std::map<std::string, double> p;
  // state
  double t = 0;                       // sim seconds
  double x = 0, y = 0, th = 0;        // world frame (bench/README.md)
  double s[2] = {0, 0};               // wheel surface speeds m/s (+ = forward)
  double v = 0, w = 0, v_prev = 0, ax_lp = 0;  // body
  double vb = 7.4, ib = 0;            // battery terminal V (PWM-averaged), current A
  double vb_min = 7.4;                // on-phase sag: both channels conducting at once
  double ich[2] = {0, 0};             // channel (= side) motor current A
  bool ilim[2] = {false, false};      // that channel is at ch_limit_a
  bool stall[2] = {false, false};     // driven but under the start/keep threshold
  double sag_ms = 0;                  // time vb_min has been below uno_resets_below_v
  bool sag = false;                   // -> Uno held in reset (vuno)
  int sag_resets = 0;                 // brownouts caused by the motors (not the fault)
  double vb_min_seen = 1e9;           // lowest vb_min since the last reset_stats()
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
    vb = vb_min = p["batt_v"];
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
  bool sag_low() const { return sag; }
  void reset_stats() { vb_min_seen = 1e9; }

  int nmot() const { int n = (int)std::lround(P("motors_per_side")); return n < 1 ? 1 : n; }
  double track_eff() const {
    double te = P("track_eff_m");
    if (te > 0) return te > 0.02 ? te : 0.02;
    double B = P("track_m"), L = P("wheelbase_m");
    if (B <= 0.02) return 0.20;
    return B * (1 + (L / B) * (L / B));
  }
  // extra volts per motor a full spin needs to drag the tyres sideways (4-wheel skid-steer)
  double scrub_geo_v() const {
    double L = P("wheelbase_m"), B = P("track_m"), mu = P("scrub_mu"), kn = P("motor_n_per_v");
    if (L <= 0 || B <= 0.02 || mu <= 0 || kn <= 1e-6) return 0;
    double f_side = mu * P("mass_kg") * 9.81 * L / (2 * B);
    return f_side / (nmot() * kn);
  }
  bool boxy() const { return P("length_m") > 0 && P("width_m") > 0; }
  double half_diag() const {
    if (!boxy()) return P("car_r_m");
    return 0.5 * std::sqrt(P("length_m") * P("length_m") + P("width_m") * P("width_m"));
  }
  // deepest penetration of the body into one wall (0 = clear), with the push-out normal
  double wall_pen(const Wall &wl, double cx, double cy, double cth, double &nx, double &ny) const {
    if (boxy()) return box_seg_pen(cx, cy, cth, P("length_m") / 2, P("width_m") / 2, wl, nx, ny);
    double R = P("car_r_m");
    double ex = wl.x2 - wl.x1, ey = wl.y2 - wl.y1;
    double L2 = ex * ex + ey * ey;
    double u = L2 > 0 ? ((cx - wl.x1) * ex + (cy - wl.y1) * ey) / L2 : 0;
    u = u < 0 ? 0 : u > 1 ? 1 : u;
    double dx = cx - (wl.x1 + u * ex), dy = cy - (wl.y1 + u * ey);
    double d = std::sqrt(dx * dx + dy * dy);
    if (d >= R) return 0;
    if (d > 1e-9) { nx = dx / d; ny = dy / d; } else { nx = -std::cos(cth); ny = -std::sin(cth); }
    return R - d;
  }

  void addSpike(double gx, double gy, double gz, double dur_s) { spikes.push_back({gx, gy, gz, t + dur_s}); }

  void step(double dt, const DriveIn in[2]) {
    t += dt;
    last_in[0] = in[0]; last_in[1] = in[1];
    const double drop0 = P("driver_drop_v"), rd = P("driver_r_ohm"), kv = P("kv_mps_per_v");
    const double rm = P("motor_r_ohm") > 0.1 ? P("motor_r_ohm") : 0.1, lim = P("ch_limit_a");
    const int n = nmot();
    bool spin = in[0].mode == 2 && in[1].mode == 2 && in[0].dir != in[1].dir;
    // how much of a spin this is: 1 = in place / one side only, 0 = straight
    double sl = in[0].mode == 2 ? in[0].dir * in[0].duty : 0, sr = in[1].mode == 2 ? in[1].dir * in[1].duty : 0;
    double f_turn = std::fabs(sl) + std::fabs(sr) > 1e-9 ? std::fabs(sl - sr) / (std::fabs(sl) + std::fabs(sr)) : 0;
    if (f_turn > 1) f_turn = 1;
    bool grounded = !f_tip && !f_lift[0] && !f_lift[1];
    double vgeo = grounded ? f_turn * scrub_geo_v() : 0;
    double tau_drive = P("tau_s") / n;
    // electrics: battery sag, driver drop and motor currents depend on each other, so solve
    // them together (a few fixed-point passes from last step's battery voltage)
    double vmk[2] = {0, 0}, emfk[2] = {0, 0}, ichk[2] = {0, 0};
    bool limk[2] = {false, false};
    double batt_avg = 0, batt_on = 0, vb_est = vb;
    for (int it = 0; it < 4; it++) {
      batt_avg = batt_on = 0;
      for (int k = 0; k < 2; k++) {
        const DriveIn &d = in[k];
        vmk[k] = 0; ichk[k] = 0; limk[k] = false;
        if (d.mode != 2) continue;
        double gain = k == 0 ? P("gain_l") : P("gain_r");
        double emf = std::fabs(s[k]) / (kv * (gain > 0.05 ? gain : 0.05));
        emfk[k] = emf;
        // drop = drop0 + rd * I_ch, I_ch = n (Vm - emf) / rm, Vm = duty (vb - drop): solve
        double vm = d.duty * (vb_est - drop0 + rd * n * emf / rm) / (1 + d.duty * rd * n / rm);
        if (vm < 0) vm = 0;
        if (lim > 0 && n * (vm - emf) / rm > lim) { vm = emf + lim * rm / n; limk[k] = true; }
        double i = (vm - emf) / rm;
        if (i < 0) i = 0;
        vmk[k] = vm; ichk[k] = n * i;
        batt_avg += d.duty * ichk[k];   // battery side of a PWM'd channel
        batt_on += ichk[k];             // on-phase: what the battery sees while EN is high
      }
      double nv = supply_v() - (batt_avg + P("idle_a")) * P("batt_r_ohm");
      vb_est = nv < 0 ? 0 : nv;
    }
    for (int k = 0; k < 2; k++) {
      const DriveIn &d = in[k];
      bool lifted = f_lift[k] || f_tip;
      double gain = k == 0 ? P("gain_l") : P("gain_r");
      double scrub = (spin ? P("scrub_v") : 0) + vgeo;
      double keep = lifted ? 0.8 : P("v_keep") + scrub;
      double start = lifted ? 0.9 : P("v_start") + scrub;
      double target = 0, tau = P("coast_tau_s");
      ilim[k] = limk[k]; stall[k] = false; ich[k] = ichk[k];
      if (d.mode == 2) {
        double vm = vmk[k];
        bool atRest = std::fabs(s[k]) < 1e-4;
        double need = atRest ? start : keep;
        if (vm >= need) target = d.dir * kv * gain * (vm - keep) * (lifted ? 1.3 : 1.0);
        else stall[k] = true;
        tau = vm >= need ? tau_drive : P("brake_tau_s") * 2;   // stalled: friction stops it
      } else if (d.mode == 1) {
        tau = P("brake_tau_s");
      }
      if (tau < 1e-3) tau = 1e-3;
      s[k] += (target - s[k]) * (dt / tau > 1 ? 1 : dt / tau);
      if (target == 0 && std::fabs(s[k]) < 2e-3) s[k] = 0;
    }
    ib = batt_avg + P("idle_a");
    vb = supply_v() - ib * P("batt_r_ohm");
    if (vb < 0) vb = 0;
    vb_min = supply_v() - (batt_on + P("idle_a")) * P("batt_r_ohm");
    if (vb_min < 0) vb_min = 0;
    if (vb_min < vb_min_seen) vb_min_seen = vb_min;
    // motor-caused brownout (the "brownout" fault is separate: browned())
    double ur = P("uno_resets_below_v");
    if (ur > 0 && vb_min < ur && !browned()) sag_ms += dt * 1000; else sag_ms = 0;
    if (!sag && ur > 0 && sag_ms >= P("brown_hold_ms")) { sag = true; sag_resets++; }
    if (sag && (ur <= 0 || vb_min >= ur + 0.1)) sag = false;   // small hysteresis

    // ground
    double g[2];
    for (int k = 0; k < 2; k++) {
      bool lifted = f_lift[k] || f_tip;
      g[k] = lifted ? 0 : s[k] * (1 - P("slip"));
      if (g[k] != 0) g[k] += P("noise_mps") * n01();
    }
    double vv = (g[0] + g[1]) / 2;
    double ww = (g[1] - g[0]) / track_eff();
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

    // walls: push out, stall the wheels when driving into one, spike on impact. A box body
    // turning in place can hit with a corner: the impact speed is that corner's (w x r).
    bool was = contact;
    contact = false;
    for (auto &wl : walls) {
      double nx = 0, ny = 0;
      double pen = wall_pen(wl, x, y, th, nx, ny);
      if (pen <= 0) continue;
      contact = true;
      x += nx * pen; y += ny * pen;
      double hx = std::cos(th), hy = std::sin(th);
      // contact point: the body's support point against the wall (-n)
      double rx = -nx * P("car_r_m"), ry = -ny * P("car_r_m");
      if (boxy()) {
        double a = P("length_m") / 2, b = P("width_m") / 2;
        double du = -(nx * hx + ny * hy), dv = -(-nx * hy + ny * hx);
        double cu = std::fabs(du) < 1e-6 ? 0 : (du > 0 ? a : -a);
        double cv = std::fabs(dv) < 1e-6 ? 0 : (dv > 0 ? b : -b);
        rx = cu * hx - cv * hy; ry = cu * hy + cv * hx;
      }
      double vpx = vv * hx + pvx - ww * ry, vpy = vv * hy + pvy + ww * rx;
      double vn = vpx * nx + vpy * ny;                           // < 0 = into the wall
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
