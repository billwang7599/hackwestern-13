// Virtual SM11: the REAL iter1-wasd/uno/tars/tars.ino (default CAR_SM11 build:
// DC wheels, EN+IN driver, kit pins), compiled unchanged for the Mac against the
// firmware test mocks (iter1-wasd/uno/test/mock), on a pseudo-terminal, driving
// physics.h. Laptop bench only; the robot never runs this.
//
//   build/vuno --profile profiles/sm11.json --truth 5590 --ctl 5591 --seed 1 \
//              --link /tmp/tars-vuno
//   tars_motor_host -b uno -d /tmp/tars-vuno       (unchanged tars_motor)
//
// Prints "PTY <path>" and "LINK <path>" on stdout once ready. Use the LINK for -d:
// after an "unplug" fault the pty is re-created under a new name and the link moved,
// as if the USB cable had been pulled and plugged back in.
//
// Like a real Uno on USB: opening the serial port (DTR) resets the board, which says
// WHEELS DC / WHEELS DCSLOW / READY boot_ms later; bytes sent meanwhile are lost.
// Truth (UDP, 50 Hz) and control (UDP) formats: bench/README.md "Virtual car".
#include "Arduino.h"
#include "Wire.h"
#include "EEPROM.h"

uint64_t mock_now_us = 1000000;
int mock_pin_input[32];
int mock_tone_hz[32];
MockEeprom EEPROM;
int mock_pin_mode[32];
int mock_pin_digital[32];
int mock_pin_pwm[32];
MockSerial Serial;
MockWire Wire;
MockMpu mock_mpu;

#include "tars.ino"

#include <arpa/inet.h>
#include <cerrno>
#include <csignal>
#include <ctime>
#include <fcntl.h>
#include <fstream>
#include <netinet/in.h>
#include <poll.h>
#include <sstream>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include "json.h"
#include "physics.h"

#if CAR != CAR_SM11
#error "the bench car is the SM11 build: don't pass -DWHEEL_TYPE/-DDRIVER_TYPE"
#endif

// ------------------------------------------------------------------ options
static std::string opt_profile, opt_link, opt_truth_host = "127.0.0.1";
static int opt_truth = 5590, opt_ctl = 5591, opt_hz = 50;
static unsigned opt_seed = 1;
static bool opt_reset_on_open = true;
static bool opt_trace = false;      // log every serial line both ways (stderr)
static std::string trace_rx, trace_tx;

static void trace(std::string &acc, const char *dir, const char *p, size_t n) {
  if (!opt_trace) return;
  for (size_t i = 0; i < n; i++) {
    char c = p[i];
    if (c == '\n' || c == '\r') {
      if (!acc.empty()) fprintf(stderr, "[vuno %lu ms] %s %s\n", millis(), dir, acc.c_str());
      acc.clear();
    } else if (acc.size() < 120) acc += c;
  }
}

static volatile sig_atomic_t g_quit = 0;
static void on_sig(int) { g_quit = 1; }

static uint64_t real_us() {
  timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (uint64_t)t.tv_sec * 1000000ull + (uint64_t)t.tv_nsec / 1000;
}

// ------------------------------------------------------------------ pty
static int pty_m = -1;
static std::string pty_path;
static bool host_open = false;      // someone (tars_motor) has the slave open
static std::string txq;             // firmware -> host bytes not yet written

static bool pty_create() {
  pty_m = posix_openpt(O_RDWR | O_NOCTTY);
  if (pty_m < 0 || grantpt(pty_m) || unlockpt(pty_m)) { perror("posix_openpt"); return false; }
  const char *n = ptsname(pty_m);
  if (!n) { perror("ptsname"); return false; }
  pty_path = n;
  fcntl(pty_m, F_SETFL, fcntl(pty_m, F_GETFL) | O_NONBLOCK);
  // Open + close the slave once: from then on poll() reports POLLHUP whenever no
  // host has it open, which is how we see tars_motor open it (= DTR = reset).
  int s = open(n, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (s >= 0) {
    termios t;
    if (tcgetattr(s, &t) == 0) { cfmakeraw(&t); cfsetspeed(&t, B115200); tcsetattr(s, TCSANOW, &t); }
    close(s);
  }
  host_open = false;
  txq.clear();
  return true;
}

static void link_update() {
  if (opt_link.empty()) return;
  std::string tmp = opt_link + ".tmp";
  unlink(tmp.c_str());
  if (pty_m >= 0) {
    if (symlink(pty_path.c_str(), tmp.c_str()) == 0) rename(tmp.c_str(), opt_link.c_str());
    else perror("symlink");
  } else {
    unlink(opt_link.c_str());   // unplugged: the device is gone
  }
}

static void pty_destroy() {
  if (pty_m >= 0) close(pty_m);
  pty_m = -1;
  host_open = false;
  txq.clear();
}

// ------------------------------------------------------------------ firmware
static Car *car;
static bool fw_running = false;      // false = held in reset / no power
static uint64_t fw_boot_at_us = 0;   // real time at which setup() runs (0 = not pending)
static int fw_boots = 0;
static uint64_t stall_n = 0, stall_max_ms = 0;   // main-loop stalls > 50 ms

static int imu_counts(double g) {
  double c = g * 4096.0;
  if (c > 32767) c = 32767;
  if (c < -32768) c = -32768;
  return (int)lround(c);
}

static void imu_to_mock() {
  mock_mpu.setAccel((int16_t)imu_counts(car->imu[0]), (int16_t)imu_counts(car->imu[1]),
                    (int16_t)imu_counts(car->imu[2]));
}

// Pins float (motor driver coasts) while the Uno is in reset.
static void pins_reset() {
  for (int i = 0; i < 32; i++) {
    mock_pin_mode[i] = -1; mock_pin_digital[i] = -1; mock_pin_pwm[i] = -1;
    mock_tone_hz[i] = 0;
  }
}

static void fw_hold_reset(uint64_t boot_after_us) {
  fw_running = false;
  pins_reset();
  fw_boot_at_us = real_us() + boot_after_us;
}

// What the firmware test harness does in boot() (test_tars.cpp), minus the EEPROM:
// a reset keeps EEPROM, like the real chip (SPEEDCAL survives a reboot).
static void fw_boot() {
  pins_reset();
  bool btn_down = mock_pin_input[PIN_BUTTON] == LOW;
  for (int i = 0; i < 32; i++) mock_pin_input[i] = HIGH;
  if (btn_down) mock_pin_input[PIN_BUTTON] = LOW;
  Serial.in.clear(); Serial.out.clear();
  int imu = (int)car->P("imu");
  mock_mpu = MockMpu();
  mock_mpu.present = imu != 0;
  if (imu == 2) mock_mpu.makeAdxl();
  imu_to_mock();
  lineLen = 0; lineOverflow = false;
  driveActive = false; lastDMs = 0;
  for (auto &a : arms) a = Arm();
  accelType = ACCEL_NONE; accelAddr = 0; accelLsbPerG = MPU_LSB_PER_G; lastAccelUs = 0;
  upX = upY = 0; upZ = 1; fX = fY = 0; fZ = 1; tiltDeg = 0;
  tipped = false; tipSinceMs = 0; uprightSinceMs = 0; lastTipMsgMs = 0;
  baselineMg = 1000; bumpWindowOpen = false; bumpPeakMg = 0; lastBumpMs = 0; bumpEverSent = false;
  btnRaw = btnStable = btnLongSent = false;
  setup();
  fw_running = true;
  fw_boot_at_us = 0;
  fw_boots++;
  fprintf(stderr, "[vuno] firmware boot #%d (tars.ino CAR_SM11)\n", fw_boots);
}

static void fw_clock() {
  static uint64_t t0 = real_us();
  uint64_t target = real_us() - t0 + 1000000ull;
  if (target > mock_now_us) mock_now_us = target;   // delay() may run ahead; never go back
}

static void read_drive(DriveIn out[2]) {
  const uint8_t EN[2] = {PIN_L_EN, PIN_R_EN}, I1[2] = {PIN_L_IN1, PIN_R_IN1}, I2[2] = {PIN_L_IN2, PIN_R_IN2};
  for (int k = 0; k < 2; k++) {
    DriveIn d;
    double en = mock_pin_pwm[EN[k]] >= 0 ? mock_pin_pwm[EN[k]] / 255.0 : mock_pin_digital[EN[k]] == HIGH ? 1.0 : 0.0;
    bool a = mock_pin_digital[I1[k]] == HIGH, b = mock_pin_digital[I2[k]] == HIGH;
    if (!fw_running || en <= 0) d.mode = 0;
    else if (a == b) d.mode = 1;
    else { d.mode = 2; d.duty = en; d.dir = a ? 1 : -1; }
    out[k] = d;
  }
}

static int pwm_signed(int k) {
  const uint8_t EN[2] = {PIN_L_EN, PIN_R_EN};
  DriveIn d = car->last_in[k];
  (void)EN;
  return d.mode == 2 ? (int)lround(d.duty * 255) * d.dir : 0;
}

// ------------------------------------------------------------------ faults / timers
static uint64_t unplug_until_us = 0;     // 0 = plugged; UINT64_MAX = until clear
static bool unplugged = false;
static uint64_t btn_release_us = 0;
static bool brown_reset = false;          // the Uno is held in reset by a brownout

static void replug() {
  unplugged = false;
  unplug_until_us = 0;
  if (pty_create()) {
    link_update();
    printf("PTY %s\n", pty_path.c_str());
    fflush(stdout);
    fprintf(stderr, "[vuno] plugged back in: %s\n", pty_path.c_str());
  }
}

static void unplug(uint64_t ms) {
  if (unplugged) return;
  pty_destroy();
  unplugged = true;
  link_update();
  unplug_until_us = ms ? real_us() + ms * 1000 : UINT64_MAX;
  fprintf(stderr, "[vuno] USB unplugged (%s)\n", ms ? "timed" : "until clear");
}

// ------------------------------------------------------------------ UDP
static int udp_out = -1, udp_ctl = -1;
struct Sub { sockaddr_in a; uint64_t until_us; };
static std::vector<Sub> subs;

static sockaddr_in addr_of(const std::string &host, int port) {
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons((uint16_t)port);
  inet_pton(AF_INET, host.c_str(), &a.sin_addr);
  return a;
}

static std::string profile_json() {
  std::ostringstream o;
  o << "{";
  bool first = true;
  for (auto &d : PARAM_DEFS) {
    if (!first) o << ",";
    first = false;
    o << "\"" << d.key << "\":" << car->P(d.key);
  }
  o << "}";
  return o.str();
}

static std::string truth_json() {
  char b[1024];
  snprintf(b, sizeof b,
           "{\"t\":%llu,\"x\":%.4f,\"y\":%.4f,\"th\":%.4f,\"v\":%.4f,\"w\":%.4f,\"wl\":%.4f,\"wr\":%.4f,"
           "\"pwm_l\":%d,\"pwm_r\":%d,\"batt_v\":%.3f,\"contact\":%s,"
           "\"batt_a\":%.3f,\"mode_l\":%d,\"mode_r\":%d,\"odo\":%.4f,\"roll\":%.1f,"
           "\"imu\":[%.3f,%.3f,%.3f],\"led\":%d,\"buzz\":%d,\"btn\":%d,"
           "\"fw\":\"%s\",\"boots\":%d,\"stalls\":%llu,\"stall_max_ms\":%llu,\"plugged\":%s,\"host_open\":%s,\"pty\":\"%s\","
           "\"faults\":{\"tip\":%s,\"lift_left\":%s,\"lift_right\":%s,\"brownout\":%s,\"unplug\":%s}}",
           (unsigned long long)(car->t * 1000), car->x, car->y, car->th, car->v, car->w, car->s[0], car->s[1],
           pwm_signed(0), pwm_signed(1), car->vb, car->contact ? "true" : "false",
           car->ib, car->last_in[0].mode, car->last_in[1].mode, car->odo_m, car->roll_deg,
           car->imu[0], car->imu[1], car->imu[2],
           fw_running && mock_pin_digital[PIN_LED] == HIGH, fw_running && mock_pin_digital[PIN_BUZZER] == HIGH,
           mock_pin_input[PIN_BUTTON] == LOW,
           fw_running ? "run" : "reset", fw_boots, (unsigned long long)stall_n, (unsigned long long)stall_max_ms, unplugged ? "false" : "true", host_open ? "true" : "false",
           jesc(unplugged ? "" : pty_path).c_str(),
           car->f_tip ? "true" : "false", car->f_lift[0] ? "true" : "false", car->f_lift[1] ? "true" : "false",
           car->browned() ? "true" : "false", unplugged ? "true" : "false");
  return b;
}

static void send_truth() {
  std::string j = truth_json();
  if (opt_truth > 0) {
    sockaddr_in a = addr_of(opt_truth_host, opt_truth);
    sendto(udp_out, j.data(), j.size(), 0, (sockaddr *)&a, sizeof a);
  }
  uint64_t now = real_us();
  std::vector<Sub> keep;
  for (auto &s : subs) {
    if (s.until_us < now) continue;
    sendto(udp_out, j.data(), j.size(), 0, (sockaddr *)&s.a, sizeof s.a);
    keep.push_back(s);
  }
  subs.swap(keep);
}

static double num_or(const JVal &o, const char *k, double d) {
  const JVal *v = o.get(k);
  return v && v->isNum() ? v->n : d;
}

static std::string handle_ctl(const std::string &msg, const sockaddr_in &from) {
  JVal j;
  if (!JParser::parse(msg, j) || j.t != JVal::OBJ) return "{\"ok\":false,\"error\":\"bad json\"}";
  std::string err, extra;
  for (auto &kv : j.o) {
    const std::string &k = kv.first;
    const JVal &v = kv.second;
    if (k == "set") {
      if (v.t != JVal::OBJ) { err = "set wants an object"; continue; }
      for (auto &s : v.o) {
        if (!Car::known(s.first)) { err += "unknown key " + s.first + "; "; continue; }
        if (!s.second.isNum()) { err += s.first + " not a number; "; continue; }
        car->p[s.first] = s.second.n;
      }
    } else if (k == "pose") {
      car->x = num_or(v, "x", car->x);
      car->y = num_or(v, "y", car->y);
      car->th = num_or(v, "th", car->th);
      car->pvx = car->pvy = 0;
    } else if (k == "walls") {
      if (v.t != JVal::ARR) { err = "walls wants [[x1,y1,x2,y2],...]"; continue; }
      std::vector<Wall> w;
      for (auto &s : v.a)
        if (s.t == JVal::ARR && s.a.size() == 4)
          w.push_back({s.a[0].num(), s.a[1].num(), s.a[2].num(), s.a[3].num()});
        else err = "bad wall entry";
      car->walls = w;
    } else if (k == "fault") {
      std::string f = v.s;
      double ms = num_or(j, "ms", -1);
      if (f == "push") {
        double bx = num_or(j, "vx", 0), by = num_or(j, "vy", 0.3);   // body frame m/s
        double c = std::cos(car->th), s = std::sin(car->th);
        car->pvx += bx * c - by * s;
        car->pvy += bx * s + by * c;
        double g = num_or(j, "g", 1.6);
        double m = std::sqrt(bx * bx + by * by);
        car->addSpike(m > 0 ? g * bx / m : 0, m > 0 ? g * by / m : 0, 0, 0.02);
      } else if (f == "bump") {
        car->addSpike(num_or(j, "g", 2.5), 0, 0, 0.02);
      } else if (f == "tip") car->f_tip = true;
      else if (f == "lift_left") car->f_lift[0] = true;
      else if (f == "lift_right") car->f_lift[1] = true;
      else if (f == "unplug") unplug(ms < 0 ? 1500 : (uint64_t)ms);
      else if (f == "brownout") {
        car->brown_v = num_or(j, "v", 4.5);
        car->brown_until = car->t + (ms < 0 ? 400 : ms) / 1000.0;
      } else if (f == "clear") {
        car->f_tip = car->f_lift[0] = car->f_lift[1] = false;
        car->brown_until = -1;
        car->spikes.clear();
        car->pvx = car->pvy = 0;
        if (unplugged) replug();
      } else err = "unknown fault " + f;
    } else if (k == "button") {
      std::string b = v.s;
      if (b == "down") { mock_pin_input[PIN_BUTTON] = LOW; btn_release_us = 0; }
      else if (b == "up") mock_pin_input[PIN_BUTTON] = HIGH;
      else if (b == "tap" || b == "hold") {
        mock_pin_input[PIN_BUTTON] = LOW;
        btn_release_us = real_us() + (b == "tap" ? 150000 : 2500000);
      } else err = "button: down|up|tap|hold";
    } else if (k == "sub") {
      int port = (int)v.num(0);
      if (port <= 0 || port > 65535) { err = "sub wants a port"; continue; }
      sockaddr_in a = from;
      a.sin_port = htons((uint16_t)port);
      bool found = false;
      for (auto &s : subs)
        if (s.a.sin_port == a.sin_port && s.a.sin_addr.s_addr == a.sin_addr.s_addr) { s.until_us = real_us() + 5000000; found = true; }
      if (!found) subs.push_back({a, real_us() + 5000000});
    } else if (k == "reset") {
      fw_hold_reset(200000);   // the RESET button: short, no host open involved
    } else if (k == "get") {
      extra += ",\"profile\":" + profile_json() + ",\"truth\":" + truth_json();
      extra += ",\"link\":\"" + jesc(opt_link) + "\"";
    } else if (k == "ms" || k == "v" || k == "vx" || k == "vy" || k == "g") {
      // fault arguments
    } else {
      err += "unknown key " + k + "; ";
    }
  }
  if (!err.empty()) return "{\"ok\":false,\"error\":\"" + jesc(err) + "\"" + extra + "}";
  return "{\"ok\":true" + extra + "}";
}

static void poll_ctl() {
  char buf[65536];
  for (int i = 0; i < 16; i++) {
    sockaddr_in from{};
    socklen_t fl = sizeof from;
    ssize_t n = recvfrom(udp_ctl, buf, sizeof buf, 0, (sockaddr *)&from, &fl);
    if (n <= 0) return;
    std::string reply = handle_ctl(std::string(buf, (size_t)n), from);
    sendto(udp_ctl, reply.data(), reply.size(), 0, (sockaddr *)&from, fl);
  }
}

// ------------------------------------------------------------------ profile
static bool load_profile(const std::string &path) {
  std::ifstream f(path);
  if (!f) { fprintf(stderr, "[vuno] can't read profile %s\n", path.c_str()); return false; }
  std::stringstream ss;
  ss << f.rdbuf();
  JVal j;
  if (!JParser::parse(ss.str(), j) || j.t != JVal::OBJ) { fprintf(stderr, "[vuno] bad JSON in %s\n", path.c_str()); return false; }
  const JVal *ps = j.get("params");
  if (!ps || ps->t != JVal::OBJ) { fprintf(stderr, "[vuno] %s: no \"params\" object\n", path.c_str()); return false; }
  for (auto &kv : ps->o) {
    if (!Car::known(kv.first)) { fprintf(stderr, "[vuno] %s: unknown param %s (ignored)\n", path.c_str(), kv.first.c_str()); continue; }
    const JVal *v = kv.second.t == JVal::OBJ ? kv.second.get("value") : &kv.second;
    if (!v || !v->isNum()) { fprintf(stderr, "[vuno] %s: %s has no numeric value\n", path.c_str(), kv.first.c_str()); return false; }
    car->p[kv.first] = v->n;
  }
  return true;
}

// ------------------------------------------------------------------ main
static void usage() {
  fprintf(stderr,
          "usage: vuno [--profile FILE] [--truth PORT (0=off)] [--truth-host IP] [--ctl PORT]\n"
          "            [--seed N] [--link PATH] [--hz N] [--no-reset-on-open] [--trace]\n");
}

int main(int argc, char **argv) {
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&]() -> std::string { if (i + 1 >= argc) { usage(); exit(2); } return argv[++i]; };
    if (a == "--profile") opt_profile = next();
    else if (a == "--truth") opt_truth = atoi(next().c_str());
    else if (a == "--truth-host") opt_truth_host = next();
    else if (a == "--ctl") opt_ctl = atoi(next().c_str());
    else if (a == "--seed") opt_seed = (unsigned)strtoul(next().c_str(), nullptr, 10);
    else if (a == "--link") opt_link = next();
    else if (a == "--hz") opt_hz = atoi(next().c_str());
    else if (a == "--no-reset-on-open") opt_reset_on_open = false;
    else if (a == "--trace") opt_trace = true;
    else { usage(); return a == "-h" || a == "--help" ? 0 : 2; }
  }
  if (opt_hz < 1 || opt_hz > 500) opt_hz = 50;
  signal(SIGINT, on_sig);
  signal(SIGTERM, on_sig);
  signal(SIGPIPE, SIG_IGN);

  car = new Car(opt_seed);
  if (!opt_profile.empty() && !load_profile(opt_profile)) return 1;
  car->vb = car->P("batt_v");

  udp_out = socket(AF_INET, SOCK_DGRAM, 0);
  udp_ctl = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in ca = addr_of("127.0.0.1", opt_ctl);
  if (bind(udp_ctl, (sockaddr *)&ca, sizeof ca) != 0) { perror("[vuno] bind control port"); return 1; }
  fcntl(udp_ctl, F_SETFL, O_NONBLOCK);

  if (!pty_create()) return 1;
  link_update();
  for (int i = 0; i < 32; i++) mock_pin_input[i] = HIGH;
  fw_clock();
  fw_boot();                       // power on (battery): nobody is listening yet
  printf("PTY %s\n", pty_path.c_str());
  if (!opt_link.empty()) printf("LINK %s\n", opt_link.c_str());
  printf("CTL %d\nTRUTH %d\n", opt_ctl, opt_truth);
  fflush(stdout);
  fprintf(stderr, "[vuno] tars.ino (CAR_SM11) on %s%s%s, truth -> %s:%d, control <- 127.0.0.1:%d, seed %u\n",
          pty_path.c_str(), opt_link.empty() ? "" : " via ", opt_link.c_str(),
          opt_truth_host.c_str(), opt_truth, opt_ctl, opt_seed);

  const double dt = 0.001;
  uint64_t sim_us = real_us();
  uint64_t next_truth = sim_us;
  const uint64_t truth_period = 1000000ull / (uint64_t)opt_hz;

  uint64_t last_iter = real_us();
  while (!g_quit) {
    uint64_t now = real_us();
    if (now - last_iter > 50000) {   // the firmware didn't run for this long: laptop busy
      stall_n++;
      uint64_t ms = (now - last_iter) / 1000;
      if (ms > stall_max_ms) stall_max_ms = ms;
      if (opt_trace || ms > 150) fprintf(stderr, "[vuno] loop stalled %llu ms (laptop busy?)\n", (unsigned long long)ms);
    }
    last_iter = now;
    fw_clock();

    // USB link: plugged? host open/close (DTR)?
    if (unplugged && unplug_until_us != UINT64_MAX && now >= unplug_until_us) replug();
    if (pty_m >= 0) {
      pollfd pf = {pty_m, POLLIN, 0};
      poll(&pf, 1, 0);
      bool open_now = !(pf.revents & POLLHUP);
      if (open_now && !host_open) {
        host_open = true;
        txq.clear();
        fprintf(stderr, "[vuno] host opened the port%s\n", opt_reset_on_open ? ": DTR reset" : "");
        if (opt_reset_on_open && !brown_reset) fw_hold_reset((uint64_t)(car->P("boot_ms") * 1000));
      } else if (!open_now && host_open) {
        host_open = false;
        txq.clear();
        fprintf(stderr, "[vuno] host closed the port\n");
      }
      if (host_open && (pf.revents & POLLIN)) {
        char b[512];
        ssize_t n = read(pty_m, b, sizeof b);
        if (n > 0) trace(trace_rx, fw_running ? "<-" : "<- (lost: in reset)", b, (size_t)n);
        if (n > 0 && fw_running) Serial.feed(std::string(b, (size_t)n));
      }
    }

    // brownout: below the Uno's threshold it sits in reset; boots when power is back
    double ur = car->P("uno_resets_below_v");
    bool low = ur > 0 && car->browned() && car->brown_v < ur;
    if (low && !brown_reset) { brown_reset = true; fw_running = false; pins_reset(); fw_boot_at_us = 0;
                               fprintf(stderr, "[vuno] brownout: Uno reset\n"); }
    if (!low && brown_reset) { brown_reset = false; fw_hold_reset(100000); }   // power-on: no Optiboot wait

    if (btn_release_us && now >= btn_release_us) { mock_pin_input[PIN_BUTTON] = HIGH; btn_release_us = 0; }

    if (!fw_running && fw_boot_at_us && now >= fw_boot_at_us && !brown_reset) fw_boot();
    if (fw_running) loop();

    // firmware output -> host
    if (!Serial.out.empty()) {
      trace(trace_tx, host_open ? "->" : "-> (nobody listening)", Serial.out.data(), Serial.out.size());
      if (host_open && pty_m >= 0) txq += Serial.out;
      Serial.out.clear();
      if (txq.size() > 8192) txq.erase(0, txq.size() - 8192);
    }
    if (!txq.empty() && pty_m >= 0 && host_open) {
      ssize_t w = write(pty_m, txq.data(), txq.size());
      if (w > 0) txq.erase(0, (size_t)w);
    }

    // physics, 1 ms steps up to real time
    DriveIn in[2];
    read_drive(in);
    int steps = 0;
    while (sim_us + 1000 <= now && steps < 100) {
      car->step(dt, in);
      sim_us += 1000;
      steps++;
    }
    if (steps == 100) sim_us = now;   // fell behind (laptop busy): don't spiral
    if (steps) imu_to_mock();

    poll_ctl();
    if (now >= next_truth) {
      send_truth();
      next_truth += truth_period;
      if (next_truth < now) next_truth = now + truth_period;
    }

    timespec ts = {0, 250000};
    nanosleep(&ts, nullptr);
  }
  pty_destroy();
  if (!opt_link.empty()) unlink(opt_link.c_str());
  fprintf(stderr, "[vuno] bye\n");
  return 0;
}
