// TARS gesture logic, C++17, header-only, no dependencies.
// Line-for-line port of vision/common/gesture.py: same labels, same rules, same
// command text. tests/test_parity.py runs both on the same inputs and diffs
// them. Change one, change the other (and vision/GESTURES.md).
#pragma once

#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace tars {

// ---- labels / intents --------------------------------------------------------
inline const char *NO_HAND = "no_hand", *UNKNOWN = "unknown", *PALM = "palm", *FIST = "fist",
                  *THUMBS_UP = "thumbs_up", *THUMBS_DOWN = "thumbs_down",
                  *POINT_UP = "point_up", *POINT_DOWN = "point_down",
                  *POINT_LEFT = "point_left", *POINT_RIGHT = "point_right", *PEACE = "peace";
inline const char *GESTURE_MODE = "gesture", *FOLLOW_MODE = "follow";
inline const char *STOP = "stop", *FORWARD = "forward", *BACK = "back",
                  *TURN_LEFT = "turn_left", *TURN_RIGHT = "turn_right";

inline std::string intent_of(const std::string &label) {
    if (label == PALM) return STOP;
    if (label == THUMBS_UP || label == POINT_UP) return FORWARD;
    if (label == FIST || label == THUMBS_DOWN || label == POINT_DOWN) return BACK;
    if (label == POINT_LEFT) return TURN_LEFT;
    if (label == POINT_RIGHT) return TURN_RIGHT;
    return "";  // none
}

// ---- config ------------------------------------------------------------------
inline const std::vector<std::string> &conf_keys() {
    static const std::vector<std::string> k = {
        "ext_ratio", "fold_ratio", "thumb_out", "dir_tolerance_deg",
        "stable_frames", "stop_frames", "lost_ms", "frame_timeout_ms", "cooldown_ms",
        "hold_repeat", "resend_ms", "step_speed", "step_ms", "turn_speed", "turn_ms",
        "point_left_is_left",
        "mode_frames", "mode_cooldown_ms",
        "follow_deadband", "follow_kp_turn", "follow_max_turn", "follow_target_size",
        "follow_size_deadband", "follow_kp_fwd", "follow_max_fwd", "follow_max_back",
        "follow_slew_per_s", "follow_send_ms", "follow_grace_frames", "follow_palm_frames",
        "follow_invert_turn",
        "flip", "infer_width",
        "min_detection_confidence", "min_tracking_confidence"};
    return k;
}

struct Config {
    std::map<std::string, double> v;
    double operator[](const std::string &k) const {
        auto it = v.find(k);
        if (it == v.end()) throw std::runtime_error("config: missing key " + k);
        return it->second;
    }
};

inline std::string trim(const std::string &s) {
    size_t a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
    return a == std::string::npos ? "" : s.substr(a, b - a + 1);
}

inline bool known_key(const std::string &k) {
    for (auto &x : conf_keys()) if (x == k) return true;
    return false;
}

// Parse gesture.conf (key = value, # comments). Every key is required.
inline Config load_config(const std::string &path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("config: cannot open " + path);
    Config c;
    std::string raw;
    int n = 0;
    while (std::getline(f, raw)) {
        n++;
        std::string line = trim(raw.substr(0, raw.find('#')));
        if (line.empty()) continue;
        size_t eq = line.find('=');
        if (eq == std::string::npos)
            throw std::runtime_error(path + ":" + std::to_string(n) + ": expected 'key = value'");
        std::string k = trim(line.substr(0, eq)), val = trim(line.substr(eq + 1));
        if (!known_key(k))
            throw std::runtime_error(path + ":" + std::to_string(n) + ": unknown key '" + k + "'");
        c.v[k] = std::stod(val);
    }
    for (auto &k : conf_keys())
        if (!c.v.count(k)) throw std::runtime_error(path + ": missing key " + k);
    return c;
}

// ---- classifier --------------------------------------------------------------
struct Pt { double x = 0, y = 0, z = 0; };
using Landmarks = std::vector<Pt>;  // 21 points, normalized to the image MediaPipe saw

constexpr int WRIST = 0, THUMB_MCP = 2, THUMB_TIP = 4, INDEX_MCP = 5, MIDDLE_MCP = 9;
constexpr int FINGERS[4][2] = {{6, 8}, {10, 12}, {14, 16}, {18, 20}};  // (pip, tip)
constexpr int PALM_POINTS[5] = {0, 5, 9, 13, 17};

inline double dist(const Landmarks &lm, int a, int b, double aspect) {
    return std::hypot((lm[a].x - lm[b].x) * aspect, lm[a].y - lm[b].y);
}

// "up" / "down" / "left" / "right" / "" for the vector a->b in the image.
inline std::string direction(const Landmarks &lm, int a, int b, double aspect, double tol) {
    double dx = (lm[b].x - lm[a].x) * aspect, dy = lm[b].y - lm[a].y;
    double ang = std::atan2(-dy, dx) * 180.0 / M_PI;  // 0 = right, 90 = up
    const std::pair<const char *, double> dirs[4] = {
        {"right", 0.0}, {"up", 90.0}, {"left", 180.0}, {"down", -90.0}};
    for (auto &d : dirs) {
        // Python's % is floored modulo; replicate it.
        double m = std::fmod(ang - d.second + 180.0, 360.0);
        if (m < 0) m += 360.0;
        if (std::fabs(m - 180.0) <= tol) return d.first;
    }
    return "";
}

inline std::string classify(const Landmarks *lmp, double aspect, const Config &cfg) {
    if (!lmp || lmp->size() < 21) return NO_HAND;
    const Landmarks &lm = *lmp;
    double palm = dist(lm, WRIST, MIDDLE_MCP, aspect);
    if (palm < 1e-6) return UNKNOWN;
    bool ext[4], fold[4];
    for (int i = 0; i < 4; i++) {
        double dp = dist(lm, WRIST, FINGERS[i][0], aspect);
        double r = dp > 1e-9 ? dist(lm, WRIST, FINGERS[i][1], aspect) / dp : 0.0;
        ext[i] = r > cfg["ext_ratio"];
        fold[i] = r < cfg["fold_ratio"];
    }
    double tol = cfg["dir_tolerance_deg"];
    if (ext[0] && ext[1] && ext[2] && ext[3]) return PALM;
    if (ext[0] && ext[1] && fold[2] && fold[3]) return PEACE;
    if (ext[0] && fold[1] && fold[2] && fold[3]) {
        std::string d = direction(lm, INDEX_MCP, 8, aspect, tol);
        if (d == "up") return POINT_UP;
        if (d == "down") return POINT_DOWN;
        if (d == "left") return POINT_LEFT;
        if (d == "right") return POINT_RIGHT;
        return UNKNOWN;
    }
    if (fold[0] && fold[1] && fold[2] && fold[3]) {
        bool thumb_out = dist(lm, THUMB_TIP, INDEX_MCP, aspect) / palm > cfg["thumb_out"];
        if (thumb_out) {
            std::string d = direction(lm, THUMB_MCP, THUMB_TIP, aspect, tol);
            if (d == "up") return THUMBS_UP;
            if (d == "down") return THUMBS_DOWN;
        }
        return FIST;
    }
    return UNKNOWN;
}

struct Feat { double cx, size; };

inline std::optional<Feat> hand_features(const Landmarks *lmp, double aspect) {
    if (!lmp || lmp->size() < 21) return std::nullopt;
    double s = 0;
    for (int i : PALM_POINTS) s += (*lmp)[i].x;
    return Feat{s / 5.0, dist(*lmp, WRIST, MIDDLE_MCP, aspect)};
}

inline double clampd(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }

inline std::pair<double, double> follow_targets(double cx, double size, const Config &c) {
    double x = c["flip"] >= 0.5 ? 1.0 - cx : cx;
    double err = x - 0.5;
    if (c["follow_invert_turn"] >= 0.5) err = -err;
    double db = c["follow_deadband"], turn = 0.0;
    if (std::fabs(err) > db) {
        turn = c["follow_kp_turn"] * (std::fabs(err) - db) * (err > 0 ? 1.0 : -1.0);
        turn = clampd(turn, -c["follow_max_turn"], c["follow_max_turn"]);
    }
    double e = c["follow_target_size"] - size, sdb = c["follow_size_deadband"], fwd = 0.0;
    if (e > sdb)
        fwd = clampd(c["follow_kp_fwd"] * (e - sdb), 0.0, c["follow_max_fwd"]);
    else if (e < -sdb)
        fwd = clampd(c["follow_kp_fwd"] * (e + sdb), -c["follow_max_back"], 0.0);
    return {clampd(fwd + turn, -1.0, 1.0), clampd(fwd - turn, -1.0, 1.0)};
}

// ---- controller --------------------------------------------------------------
struct Cmd {
    std::string text;    // "D 0.50 0.50" or "S"
    std::string reason;  // see gesture.py Cmd
};

inline std::string fmt_d(double l, double r) {
    char b[48];
    std::snprintf(b, sizeof b, "D %.2f %.2f", l, r);
    return b;
}

inline double toward(double cur, double tgt, double step) {
    double d = tgt - cur;
    return std::fabs(d) <= step ? tgt : cur + (d > 0 ? step : -step);
}

class Controller {
public:
    explicit Controller(Config cfg) : c(std::move(cfg)) {}

    std::string mode = GESTURE_MODE;
    std::string cand = NO_HAND;
    int cand_count = 0, miss = 0;
    std::optional<double> last_hand_t, last_frame_t;
    bool need_stop = false;
    // gesture mode
    bool has_move = false;
    std::string move_intent;
    double move_l = 0, move_r = 0, move_end = 0;
    double last_send = -1e18, cooldown_until = -1e18;
    std::string fired_intent;  // "" = none
    bool armed = true;
    // toggle
    bool toggle_armed = true;
    double mode_cooldown_until = -1e18;
    // follow
    bool f_active = false;
    double f_tgt_l = 0, f_tgt_r = 0, f_cur_l = 0, f_cur_r = 0;

    const Config &cfg() const { return c; }

    std::vector<Cmd> on_frame(double t, const std::string &label, std::optional<Feat> feat) {
        std::vector<Cmd> out;
        last_frame_t = t;
        if (label == NO_HAND) {
            cand = NO_HAND;
            cand_count = 0;
            miss++;
        } else {
            miss = 0;
            last_hand_t = t;
            if (label == cand) cand_count++;
            else { cand = label; cand_count = 1; }
        }
        if (label == PEACE) {
            if (cand_count >= c["mode_frames"] && toggle_armed && t >= mode_cooldown_until) {
                std::string nm = mode == GESTURE_MODE ? FOLLOW_MODE : GESTURE_MODE;
                append(out, stop(t, "mode_" + nm));
                mode = nm;
                toggle_armed = false;
                mode_cooldown_until = t + c["mode_cooldown_ms"];
                cooldown_until = t + c["cooldown_ms"];
                fired_intent.clear();
                armed = true;
                append(out, tick(t));
                return out;
            }
        } else {
            toggle_armed = true;
        }
        if (mode == FOLLOW_MODE) append(out, follow_frame(t, label, feat));
        else append(out, gesture_frame(t, label));
        append(out, tick(t));
        return out;
    }

    std::vector<Cmd> tick(double t) {
        if (last_frame_t && t - *last_frame_t >= c["frame_timeout_ms"] &&
            (need_stop || has_move || f_active))
            return stop(t, "camera_timeout");
        if (mode == FOLLOW_MODE) {
            if (!f_active || t - last_send < c["follow_send_ms"]) return {};
            double dt = std::fmin(t - last_send, 2 * c["follow_send_ms"]) / 1000.0;
            last_send = t;
            double step = c["follow_slew_per_s"] * dt;
            f_cur_l = toward(f_cur_l, f_tgt_l, step);
            f_cur_r = toward(f_cur_r, f_tgt_r, step);
            need_stop = true;
            return {Cmd{fmt_d(f_cur_l, f_cur_r), "follow"}};
        }
        if (!has_move) return {};
        if (t >= move_end) {
            auto out = stop(t, "done");
            cooldown_until = t + c["cooldown_ms"];
            return out;
        }
        if (t - last_send >= c["resend_ms"]) {
            last_send = t;
            return {Cmd{fmt_d(move_l, move_r), "resend"}};
        }
        return {};
    }

private:
    Config c;

    static void append(std::vector<Cmd> &a, const std::vector<Cmd> &b) {
        a.insert(a.end(), b.begin(), b.end());
    }

    void wheels(const std::string &intent, double &l, double &r, double &dur) const {
        double s = c["step_speed"], ts = c["turn_speed"];
        if (intent == FORWARD) { l = s; r = s; dur = c["step_ms"]; return; }
        if (intent == BACK) { l = -s; r = -s; dur = c["step_ms"]; return; }
        bool ccw = (intent == TURN_LEFT) == (c["point_left_is_left"] >= 0.5);
        if (ccw) { l = -ts; r = ts; } else { l = ts; r = -ts; }
        dur = c["turn_ms"];
    }

    std::vector<Cmd> stop(double t, const std::string &reason) {
        if (has_move) cooldown_until = t + c["cooldown_ms"];
        has_move = false;
        need_stop = false;
        f_active = false;
        f_tgt_l = f_tgt_r = f_cur_l = f_cur_r = 0.0;
        return {Cmd{"S", reason}};
    }

    std::vector<Cmd> gesture_frame(double t, const std::string &label) {
        if (label == NO_HAND) {
            armed = true;
            bool lost = !last_hand_t || t - *last_hand_t >= c["lost_ms"];
            if (lost && (need_stop || has_move)) return stop(t, "lost");
            return {};
        }
        std::string intent = intent_of(label);
        if (intent != fired_intent) armed = true;
        if (intent == STOP) {
            bool first = cand_count == (int)c["stop_frames"];
            if (first || (cand_count > c["stop_frames"] && (need_stop || has_move))) {
                auto out = stop(t, "palm");
                cooldown_until = t + c["cooldown_ms"];
                return out;
            }
        } else if (!intent.empty()) {
            bool ready = cand_count >= c["stable_frames"] && !has_move && t >= cooldown_until &&
                         (armed || c["hold_repeat"] >= 0.5);
            if (ready) {
                double l, r, dur;
                wheels(intent, l, r, dur);
                has_move = true;
                move_intent = intent;
                move_l = l; move_r = r; move_end = t + dur;
                last_send = t;
                need_stop = true;
                fired_intent = intent;
                armed = false;
                return {Cmd{fmt_d(l, r), intent}};
            }
        }
        return {};
    }

    std::vector<Cmd> follow_frame(double t, const std::string &label, std::optional<Feat> feat) {
        if (label == NO_HAND) {
            if (miss >= c["follow_grace_frames"] && (need_stop || f_active)) return stop(t, "lost");
            return {};
        }
        if (label == PALM) {
            int n = cand_count;
            if (n >= c["follow_palm_frames"] &&
                (n == (int)c["follow_palm_frames"] || need_stop || f_active))
                return stop(t, "palm");
            return {};
        }
        if (label == PEACE) {
            f_tgt_l = f_tgt_r = 0.0;
            return {};
        }
        if (feat) {
            auto lr = follow_targets(feat->cx, feat->size, c);
            f_tgt_l = lr.first;
            f_tgt_r = lr.second;
            if (!f_active) {
                f_active = true;
                last_send = t - c["follow_send_ms"];
            }
        }
        return {};
    }
};

}  // namespace tars
