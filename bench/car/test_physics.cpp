// Unit tests for physics.h alone (no firmware, no pty): 2WD vs 4WD electrics, the per-channel
// current limit, battery sag and the motor-caused Uno brownout, tyre scrub in a spin, and the
// rectangular body against walls. Laptop only; every number comes from the guessed profiles.
//
//   make -C bench/car test        (builds build/test_physics and runs it, with the grid)
//   build/test_physics [--grid] [--profiles DIR]
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#include "json.h"
#include "physics.h"

static int fails = 0, total = 0;
static void check(bool ok, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void check(bool ok, const char *fmt, ...) {
  total++;
  if (!ok) fails++;
  printf(ok ? "  ok   " : "  FAIL ");
  va_list ap;
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
  printf("\n");
}

static std::string g_dir = "profiles";

static void load(Car &c, const char *name) {
  std::ifstream f(g_dir + "/" + name + ".json");
  std::stringstream ss;
  ss << f.rdbuf();
  JVal j;
  if (!JParser::parse(ss.str(), j) || !j.get("params")) { fprintf(stderr, "bad profile %s\n", name); exit(2); }
  for (auto &kv : j.get("params")->o) {
    const JVal *v = kv.second.t == JVal::OBJ ? kv.second.get("value") : &kv.second;
    if (!Car::known(kv.first)) { fprintf(stderr, "%s: unknown param %s\n", name, kv.first.c_str()); exit(2); }
    c.p[kv.first] = v->n;
  }
  c.vb = c.vb_min = c.P("batt_v");
}

static DriveIn drv(double signed_duty) {
  DriveIn d;
  if (signed_duty == 0) { d.mode = 1; return d; }   // brake, like the firmware's stop
  d.mode = 2; d.duty = std::fabs(signed_duty); d.dir = signed_duty > 0 ? 1 : -1;
  return d;
}

// Run for secs at signed duties (l, r). kick_ms > 0: the firmware's DC kick first
// (DC_KICK_DUTY 0.8 for DC_KICK_MS 40, same signs), as tars.ino does from rest.
struct Run { double dth_deg, dist_m, vb_min, ich_max, w_end_dps, v_end, imu_xy_max; bool ilim, sag, contact; int sag_resets; };
static Run run(Car &c, double l, double r, double secs, int kick_ms = 0, double kick = 0.8) {
  Run o{};
  double th0 = c.th, x0 = c.x, y0 = c.y, acc = 0, prev = c.th;
  o.vb_min = 1e9;
  int r0 = c.sag_resets;
  int steps = (int)(secs * 1000);
  for (int i = 0; i < steps; i++) {
    double dl = l, dr = r;
    if (i < kick_ms) { dl = l > 0 ? kick : l < 0 ? -kick : 0; dr = r > 0 ? kick : r < 0 ? -kick : 0; }
    DriveIn in[2] = {drv(dl), drv(dr)};
    c.step(0.001, in);
    acc += std::atan2(std::sin(c.th - prev), std::cos(c.th - prev));
    prev = c.th;
    if (c.vb_min < o.vb_min) o.vb_min = c.vb_min;
    for (int k = 0; k < 2; k++) { if (c.ich[k] > o.ich_max) o.ich_max = c.ich[k]; if (c.ilim[k]) o.ilim = true; }
    if (c.sag_low()) o.sag = true;
    if (c.contact) o.contact = true;
    double h = std::sqrt(c.imu[0] * c.imu[0] + c.imu[1] * c.imu[1]);
    if (h > o.imu_xy_max) o.imu_xy_max = h;
  }
  o.dth_deg = acc * 180 / M_PI;
  o.dist_m = (c.x - x0) * std::cos(th0) + (c.y - y0) * std::sin(th0);
  o.w_end_dps = c.w * 180 / M_PI;
  o.v_end = c.v;
  o.sag_resets = c.sag_resets - r0;
  return o;
}

// firmware TURN at D 1 = kick, then DC_MAX_DUTY 0.45; slow TURN = kick, then 0.36
static Run fw_turn(Car &c, double duty, double secs) { return run(c, duty, -duty, secs, 40); }

static void grid_print() {
  printf("\n  brownout grid: lowest on-phase battery V (Uno resets below 6.0 V for 5 ms in sm11.json)\n");
  printf("  %-5s %-6s %-6s | %-18s | %-18s | %-18s\n", "car", "batt", "R_int", "TURN (0.8 kick,.45)",
         "spin at duty 1.0", "stalled @0.8 (wall)");
  for (int wd : {2, 4})
    for (double bv : {8.4, 7.4, 7.0})
      for (double br : {0.2, 0.3, 0.5}) {
        Run rr[3];
        for (int k = 0; k < 3; k++) {
          Car c(1);
          load(c, wd == 4 ? "sm11" : "sm11_2wd");
          c.p["batt_v"] = bv; c.p["batt_r_ohm"] = br; c.vb = c.vb_min = bv;
          if (k == 0) rr[k] = fw_turn(c, 0.45, 0.6);
          else if (k == 1) rr[k] = run(c, 1.0, -1.0, 0.6);
          else { double xw = c.P("length_m") > 0 ? c.P("length_m") / 2 + 0.0075 : 0.135;
                 c.walls = {{xw, -1, xw, 1}, {-xw, -1, -xw, 1}}; rr[k] = run(c, 0.8, 0.8, 0.6); }
        }
        printf("  %dWD   %.1f V  %.1f ohm | %5.2f V %-11s| %5.2f V %-11s| %5.2f V %-11s\n", wd, bv, br,
               rr[0].vb_min, rr[0].sag ? "RESET" : "", rr[1].vb_min, rr[1].sag ? "RESET" : "",
               rr[2].vb_min, rr[2].sag ? "RESET" : "");
      }
}

int main(int argc, char **argv) {
  bool grid = false;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--grid")) grid = true;
    else if (!strcmp(argv[i], "--profiles") && i + 1 < argc) g_dir = argv[++i];
  }

  printf("defaults (no profile)\n");
  {
    Car c(1);
    check(c.nmot() == 1 && !c.boxy() && std::fabs(c.track_eff() - 0.20) < 1e-12 && c.scrub_geo_v() == 0,
          "1 motor/side, circle body, track_eff 0.20, no tyre scrub: old profiles behave as before");
    Run a = run(c, 0.45, 0.45, 2.0, 40);
    check(a.v_end > 0.13 && a.v_end < 0.17, "duty 0.45 straight = %.0f mm/s (the firmware assumes ~150)", a.v_end * 1000);
  }

  printf("1 vs 2 motors per side (sm11.json, only motors_per_side changed)\n");
  {
    Car a(1), b(1);
    load(a, "sm11"); load(b, "sm11");
    a.p["motors_per_side"] = 1;
    Run ra = run(a, 0.45, 0.45, 2.0, 40), rb = run(b, 0.45, 0.45, 2.0, 40);
    check(rb.v_end < ra.v_end - 0.01 && rb.v_end > 0.8 * ra.v_end,
          "straight at 0.45: 4WD %.0f vs 2WD %.0f mm/s (same motors; 2x current = more L298N drop + sag)",
          rb.v_end * 1000, ra.v_end * 1000);
    double ia = a.ich[0], ib = b.ich[0];
    check(ib > 1.7 * ia && ib < 2.3 * ia, "straight at 0.45: channel current %.2f A with 2 motors vs %.2f A with 1 (~2x)", ib, ia);
    check(b.vb < a.vb - 0.03, "straight at 0.45: battery %.2f V with 4 motors vs %.2f V with 2 (more sag)", b.vb, a.vb);
    check(b.ib > 1.5 * a.ib, "battery current %.2f A (4 motors) vs %.2f A (2)", b.ib, a.ib);
    Car s1(1), s2(1);
    load(s1, "sm11"); load(s2, "sm11");
    s1.p["motors_per_side"] = 1;
    const double xw = s2.P("length_m") / 2 + 0.0075;   // a wall 7.5 mm ahead of the nose
    for (Car *c : {&s1, &s2}) c->walls = {{xw, -1, xw, 1}};
    Run t1 = run(s1, 1.0, 1.0, 0.5), t2 = run(s2, 1.0, 1.0, 0.5);
    check(t2.ich_max > 1.6 * t1.ich_max, "stalled against a wall at duty 1: channel %.2f A (2 motors) vs %.2f A (1)",
          t2.ich_max, t1.ich_max);
    check(!t2.ilim && t2.ich_max < 2.0, "4WD stall %.2f A/channel stays under the L298N's 2 A (no limiting)", t2.ich_max);
    check(std::fabs(b.scrub_geo_v() * 2 - a.scrub_geo_v()) < 1e-9,
          "spin scrub per motor: %.2f V with 2 motors vs %.2f V with 1 (twice the torque per side)", b.scrub_geo_v(), a.scrub_geo_v());
  }

  printf("per-channel current limit\n");
  {
    Car c(1);
    load(c, "sm11");
    c.p["ch_limit_a"] = 0.6;   // an L293D-style board
    c.walls = {{c.P("length_m") / 2 + 0.0075, -1, c.P("length_m") / 2 + 0.0075, 1}};
    Run r = run(c, 1.0, 1.0, 0.5);
    check(r.ilim && r.ich_max <= 0.6 + 1e-9, "ch_limit_a 0.6 (L293D-style): 4WD stall clamped to %.3f A, ilim flagged", r.ich_max);
    Car d(1);
    load(d, "sm11");
    d.p["ch_limit_a"] = 0.6;
    Run s = run(d, 1.0, -1.0, 1.0);
    Car e(1);
    load(e, "sm11");
    Run u = run(e, 1.0, -1.0, 1.0);
    check(std::fabs(s.dth_deg) < std::fabs(u.dth_deg),
          "the limit costs torque: duty-1 spin turns %.0f deg in 1 s at 0.6 A vs %.0f deg at 2 A", std::fabs(s.dth_deg), std::fabs(u.dth_deg));
  }

  printf("skid-steer: tyre scrub, low-duty spin stall, effective track\n");
  double min_spin_duty = 0;
  {
    Car c(1);
    load(c, "sm11");
    const double B = c.P("track_m"), L = c.P("wheelbase_m");
    check(B > 0 && L > 0 && std::fabs(c.track_eff() - B * (1 + std::pow(L / B, 2))) < 1e-9,
          "4WD track_eff derived from geometry: %.3f m (track %.3f, wheelbase %.3f)", c.track_eff(), B, L);
    printf("  ..   4WD spin needs %.2f V/motor extra (tyre scrub) + scrub_v %.2f over v_keep %.2f / v_start %.2f\n",
           c.scrub_geo_v(), c.P("scrub_v"), c.P("v_keep"), c.P("v_start"));
    Car s(1);
    load(s, "sm11");
    Run slow = fw_turn(s, 0.36, 1.5);
    check(std::fabs(slow.dth_deg) < 5, "4WD slow TURN (kick, then 0.36): turns %.1f deg in 1.5 s = STALLS", -slow.dth_deg);
    Car f(1);
    load(f, "sm11");
    Run d1 = fw_turn(f, 0.45, 1.5);
    printf("  ..   4WD TURN at D 1 (kick, then 0.45): %.1f deg in 1.5 s\n", -d1.dth_deg);
    Car g(1);
    load(g, "sm11");
    Run full = run(g, 1.0, -1.0, 1.5);
    check(std::fabs(full.dth_deg) > 90, "4WD spin at duty 1.0: %.0f deg in 1.5 s (%.0f deg/s) = turns", -full.dth_deg, -full.w_end_dps);
    for (double du = 0.30; du <= 1.0001; du += 0.01) {
      Car h(1);
      load(h, "sm11");
      Run q = fw_turn(h, du, 1.5);
      if (std::fabs(q.dth_deg) > 30) { min_spin_duty = du; break; }
    }
    check(min_spin_duty > 0.45, "4WD lowest firmware spin duty that turns (>30 deg in 1.5 s, after the kick): %.2f "
          "(firmware cap DC_MAX_DUTY 0.45, slow 0.36)", min_spin_duty);
    Car w2(1);
    load(w2, "sm11_2wd");
    Run two = fw_turn(w2, 0.36, 1.5);
    check(std::fabs(two.dth_deg) > 30, "2WD slow TURN (kick, then 0.36): turns %.0f deg in 1.5 s", -two.dth_deg);
    Car ln(1);
    load(ln, "sm11");
    ln.p["mass_kg"] = 0.45; ln.p["scrub_mu"] = 0.3;   // bare kit on smooth tile
    Run light = fw_turn(ln, 0.45, 1.5);
    printf("  ..   4WD bare kit (0.45 kg, mu 0.3) TURN at D 1: %.0f deg in 1.5 s\n", -light.dth_deg);
    // sm11_taped.json: rear tyres wrapped in tape (scrub_mu = the rear pair's sliding friction)
    Car tp(1), tq(1);
    load(tp, "sm11_taped"); load(tq, "sm11_taped");
    Run tf = fw_turn(tp, 0.45, 1.5), ts = fw_turn(tq, 0.36, 1.5);
    double taped_min = 0;
    for (double du = 0.30; du <= 1.0001; du += 0.01) {
      Car h(1);
      load(h, "sm11_taped");
      Run q = fw_turn(h, du, 1.5);
      if (std::fabs(q.dth_deg) > 30) { taped_min = du; break; }
    }
    printf("  ..   taped rears (mu %.2f): scrub %.2f V/motor; TURN at D 1 %.0f deg, slow %.0f deg in 1.5 s; lowest spin duty %.2f\n",
           tp.P("scrub_mu"), tp.scrub_geo_v(), -tf.dth_deg, -ts.dth_deg, taped_min);
    check(std::fabs(tf.dth_deg) > 10 && std::fabs(tf.dth_deg) > std::fabs(d1.dth_deg) + 10 && taped_min < min_spin_duty,
          "taped rear tyres: the firmware's TURN (kick, then 0.45) now creeps round %.0f deg in 1.5 s (untaped %.0f); "
          "lowest spin duty %.2f vs %.2f untaped: marginal at the 0.45 cap", -tf.dth_deg, -d1.dth_deg, taped_min, min_spin_duty);
    Car st(1);
    load(st, "sm11");
    Run straight = run(st, 0.36, 0.36, 1.5, 40);
    check(straight.dist_m > 0.08, "4WD straight at 0.36 still rolls (%.0f mm in 1.5 s): only spins scrub", straight.dist_m * 1000);
  }

  printf("battery sag and the motor-caused Uno brownout\n");
  {
    Car c(1);
    load(c, "sm11");
    Run r = run(c, 1.0, -1.0, 0.5);
    printf("  ..   4WD full-duty (1.0) spin start from rest, 7.4 V / 0.30 ohm: battery min %.2f V, %.2f A/channel peak\n",
           r.vb_min, r.ich_max);
    check(r.sag == (r.vb_min < 6.0), "sag flag agrees with the 6.0 V threshold (min %.2f V -> %s)", r.vb_min, r.sag ? "RESET" : "no reset");
    Car k(1);
    load(k, "sm11");
    Run kick = fw_turn(k, 0.45, 0.5);
    printf("  ..   4WD firmware TURN (0.8 kick): battery min %.2f V -> %s\n", kick.vb_min, kick.sag ? "RESET" : "no reset");
    Car lo(1), lo2(1);
    load(lo, "sm11"); load(lo2, "sm11_2wd");
    for (Car *x : {&lo, &lo2}) { x->p["batt_v"] = 7.0; x->p["batt_r_ohm"] = 0.5; x->vb = x->vb_min = 7.0; }
    Run a = run(lo, 1.0, -1.0, 0.5), b = run(lo2, 1.0, -1.0, 0.5);
    check(a.sag && a.sag_resets == 1, "flat battery (7.0 V, 0.5 ohm): 4WD duty-1 spin start browns out the Uno (min %.2f V)", a.vb_min);
    check(!b.sag && b.vb_min > a.vb_min + 0.3, "same battery, 2WD: no reset (min %.2f V)", b.vb_min);
    run(lo, 0, 0, 0.1);
    check(!lo.sag_low(), "sag clears once the motors stop (battery %.2f V)", lo.vb_min);
    Car z(1);
    load(z, "sm11");
    z.p["uno_resets_below_v"] = 0; z.p["batt_v"] = 6.0; z.vb = z.vb_min = 6.0;
    Run q = run(z, 1.0, -1.0, 0.5);
    check(!q.sag, "uno_resets_below_v 0 (Uno powered by the Pi's USB): never resets (min %.2f V)", q.vb_min);
    Car sh(1);
    load(sh, "sm11");
    sh.p["batt_v"] = 7.0; sh.p["batt_r_ohm"] = 0.5; sh.p["brown_hold_ms"] = 1000; sh.vb = sh.vb_min = 7.0;
    Run hold = run(sh, 1.0, -1.0, 0.5);
    check(!hold.sag, "a dip shorter than brown_hold_ms doesn't reset");
  }

  printf("rectangular body vs walls\n");
  {
    Car c(1);
    load(c, "sm11");
    const double hl = c.P("length_m") / 2, hw = c.P("width_m") / 2;
    c.walls = {{-1, 0.12, 1, 0.12}};   // wall alongside: ~3 cm off the wheels, inside the half-diagonal
    Run idle = run(c, 0, 0, 0.1);
    check(hw < 0.12 && c.half_diag() > 0.12 + 0.005, "geometry puts 0.12 m between half-width %.4f and half-diagonal %.4f", hw, c.half_diag());
    check(!idle.contact, "box %.3f x %.3f parked 0.12 m from a wall: no contact (half-width %.4f)", 2 * hl, 2 * hw, hw);
    Run sp = run(c, 1.0, -1.0, 1.0);
    check(sp.contact && c.y < -0.004, "spin in place there: a corner (half-diagonal %.3f) hits it, car shoved %.0f mm away",
          c.half_diag(), -c.y * 1000);
    check(sp.imu_xy_max > 1.2, "the corner strike reads %.1f g on the IMU (firmware bump threshold 1.2 g)", sp.imu_xy_max);
    Car o(1);
    load(o, "sm11");
    o.p["length_m"] = 0; o.p["car_r_m"] = 0.11;
    o.walls = {{-1, 0.12, 1, 0.12}};
    Run sc = run(o, 1.0, -1.0, 1.0);
    check(!sc.contact, "the same spin with the old 0.11 m circle: no contact (what a circle body misses)");
    Car h(1);
    load(h, "sm11");
    h.walls = {{0.5, -1, 0.5, 1}};
    run(h, 0.45, 0.45, 5.0, 40);
    check(h.contact && std::fabs(h.x - (0.5 - hl)) < 0.005, "head-on: stops with the nose on the wall (centre x %.4f, expect %.4f)",
          h.x, 0.5 - hl);
    Car bx(1);
    load(bx, "sm11");
    bx.th = M_PI / 4;   // 45 deg toward a wall at x = 0.3: a corner leads
    bx.walls = {{0.3, -1, 0.3, 1}};
    run(bx, 0.6, 0.6, 3.0, 40);
    double a = hl, b = hw, cth = std::cos(bx.th), sth = std::sin(bx.th);
    double reach = a * std::fabs(cth) + b * std::fabs(sth);
    check(bx.contact && std::fabs(bx.x + reach - 0.3) < 0.006, "approach at an angle: stops when the leading corner touches (corner x %.4f)",
          bx.x + reach);
  }

  if (grid) grid_print();
  printf("%d/%d checks passed\n", total - fails, total);
  return fails ? 1 : 0;
}
