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
                  *POINT_LEFT = "point_left", *POINT_RIGHT = "point_right", *PEACE = "peace",
                  *TEACH = "teach";
inline const char *GESTURE_MODE = "gesture", *FOLLOW_MODE = "follow", *PET_MODE = "pet";
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
        "min_detection_confidence", "min_tracking_confidence",
        "spin_turns", "wiggle_ms", "arms_ms",
        "teach_hold_ms", "teach_countdown_ms", "learn_gap_frames", "learn_cancel_frames",
        "learn_frames", "learn_record_timeout_ms", "learn_assign_ms", "learn_assign_frames",
        "learn_cooldown_ms", "learn_builtin_frac", "learn_similar_frac", "learn_max_classes",
        "learn_k", "learn_augment", "learn_jitter", "learn_angle_weight",
        "learn_thr_mult", "learn_thr_min", "learn_thr_max",
        "feedback_speed", "feedback_ms", "nudge_ms", "feedback_led_beep",
        "pet_in_cycle", "pet_start", "pet_exit_ms", "pet_trick_frames", "pet_gap_frames",
        "pet_sleep_frames", "pet_wave_window_ms", "pet_wave_amp", "pet_wave_swings",
        "pet_send_ms", "pet_settle_ms", "pet_come_size", "pet_come_max_fwd", "pet_come_max_ms",
        "pet_body_ms", "pet_happy_ms", "pet_sad_ms", "pet_look_ms", "pet_max_seg_ms", "pet_max_speed",
        "pet_bored_ms", "pet_look_every_ms", "pet_sleep_after_ms",
        "pet_energy_start", "pet_energy_rest", "pet_energy_decay_per_s", "pet_happy_boost",
        "pet_greet_boost", "pet_come_boost", "pet_sad_drop", "pet_excited",
        "pet_speed_min", "pet_speed_max", "pet_seed"};
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
    if (fold[0] && fold[1] && fold[2] && ext[3]) return TEACH;
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

// Held peace sign cycles GESTURE -> FOLLOW -> PET (if pet_in_cycle) -> GESTURE.
inline std::string next_mode(const std::string &mode, const Config &c) {
    if (mode == GESTURE_MODE) return FOLLOW_MODE;
    if (mode == FOLLOW_MODE && c["pet_in_cycle"] >= 0.5) return PET_MODE;
    return GESTURE_MODE;
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

// A move is a list of segments: D drives (resent every resend_ms), X sends one
// line (e.g. "G wave", "BEEP 1") and waits ms. Same as gesture.py.
struct Seg {
    bool d = true;
    double l = 0, r = 0, ms = 0;
    std::string text;
};
inline Seg segD(double l, double r, double ms) { return Seg{true, l, r, ms, ""}; }
inline Seg segX(const std::string &text, double ms) { return Seg{false, 0, 0, ms, text}; }

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
    // multi-segment moves + learned poses
    std::vector<Seg> segs;   // still to run after the current one
    bool move_d = true;
    std::map<std::string, std::pair<std::string, std::vector<Seg>>> learned;  // label -> (action, custom)

    const Config &cfg() const { return c; }

    std::string intent_of(const std::string &label) const {
        std::string it = tars::intent_of(label);
        if (it.empty()) {
            auto f = learned.find(label);
            if (f != learned.end()) it = f->second.first + "@" + label;
        }
        return it;
    }

    std::vector<Seg> action_segments(const std::string &action, const std::vector<Seg> &custom = {}) const {
        double s = c["step_speed"], ts = c["turn_speed"];
        if (action == FORWARD) return {segD(s, s, c["step_ms"])};
        if (action == BACK) return {segD(-s, -s, c["step_ms"])};
        if (action == TURN_LEFT || action == TURN_RIGHT) {
            bool ccw = (action == TURN_LEFT) == (c["point_left_is_left"] >= 0.5);
            if (ccw) return {segD(-ts, ts, c["turn_ms"])};
            return {segD(ts, -ts, c["turn_ms"])};
        }
        if (action == "spin") return {segD(-ts, ts, c["turn_ms"] * c["spin_turns"])};
        if (action == "wiggle") {
            double w = c["wiggle_ms"];
            return {segD(-ts, ts, w), segD(ts, -ts, 2 * w), segD(-ts, ts, 2 * w), segD(ts, -ts, 2 * w),
                    segD(-ts, ts, w)};
        }
        if (action == "arms") return {segX("G wave", c["arms_ms"])};
        if (action == "custom") return custom;
        return {};
    }

    // Run a segment list now (cancels any move). Used for actions + teach feedback.
    std::vector<Cmd> play(double t, const std::string &intent, const std::vector<Seg> &s) {
        std::vector<Cmd> out;
        if (has_move || need_stop || f_active) append(out, stop(t, "preempt"));
        segs = s;
        append(out, start_seg(t, intent));
        return out;
    }

    // public so the teach layer (learn.hpp) can stop like gesture.py's _stop
    std::vector<Cmd> stop_now(double t, const std::string &reason) { return stop(t, reason); }

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
        // In PET mode the pet layer owns the peace sign (flash = trick, hold = leave).
        if (label == PEACE) {
            if (mode != PET_MODE && cand_count >= c["mode_frames"] && toggle_armed && t >= mode_cooldown_until) {
                std::string nm = next_mode(mode, c);
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
        else if (mode == PET_MODE) append(out, pet_frame(t, label));
        else append(out, gesture_frame(t, label));
        append(out, tick(t));
        return out;
    }

    std::vector<Cmd> tick(double t) {
        if (last_frame_t && t - *last_frame_t >= c["frame_timeout_ms"] &&
            (need_stop || has_move || f_active))
            return stop(t, "camera_timeout");
        if (mode == FOLLOW_MODE || (mode == PET_MODE && f_active)) {
            bool pet = mode == PET_MODE;
            double send_ms = pet ? c["pet_send_ms"] : c["follow_send_ms"];
            if (!f_active || t - last_send < send_ms) return {};
            double dt = std::fmin(t - last_send, 2 * send_ms) / 1000.0;
            last_send = t;
            double step = c["follow_slew_per_s"] * dt;
            f_cur_l = toward(f_cur_l, f_tgt_l, step);
            f_cur_r = toward(f_cur_r, f_tgt_r, step);
            need_stop = true;
            return {Cmd{fmt_d(f_cur_l, f_cur_r), pet ? "pet_stream" : "follow"}};
        }
        if (!has_move) return {};
        if (t >= move_end) return start_seg(t, move_intent);
        if (move_d && t - last_send >= c["resend_ms"]) {
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

    std::vector<Seg> segments_for(const std::string &intent) const {
        size_t at = intent.find('@');
        if (at != std::string::npos) {
            std::string act = intent.substr(0, at), label = intent.substr(at + 1);
            auto f = learned.find(label);
            static const std::vector<Seg> none;
            return action_segments(act, f != learned.end() ? f->second.second : none);
        }
        return action_segments(intent);
    }

    std::vector<Cmd> start_seg(double t, const std::string &intent) {
        std::vector<Cmd> out;
        while (!segs.empty()) {
            Seg s = segs.front();
            segs.erase(segs.begin());
            has_move = true;
            move_intent = intent;
            move_l = s.d ? s.l : 0.0;
            move_r = s.d ? s.r : 0.0;
            move_end = t + s.ms;
            move_d = s.d;
            last_send = t;
            need_stop = true;
            out.push_back(Cmd{s.d ? fmt_d(s.l, s.r) : s.text, intent});
            if (s.ms > 0) return out;
        }
        append(out, stop(t, "done"));
        cooldown_until = t + c["cooldown_ms"];
        return out;
    }

    std::vector<Cmd> stop(double t, const std::string &reason) {
        if (has_move) cooldown_until = t + c["cooldown_ms"];
        has_move = false;
        segs.clear();
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
        std::string intent = this->intent_of(label);
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
                fired_intent = intent;
                armed = false;
                segs = segments_for(intent);
                return start_seg(t, intent);
            }
        }
        return {};
    }

    // Safety floor under PET mode; pet.hpp decides the behaviour (see gesture.py).
    std::vector<Cmd> pet_frame(double t, const std::string &label) {
        if (label == NO_HAND) {
            if (miss >= c["follow_grace_frames"] && f_active) return stop(t, "lost");
            return {};
        }
        if (label == PALM) {
            int n = cand_count;
            if (n == (int)c["stop_frames"] || (n > c["stop_frames"] && (need_stop || has_move || f_active)))
                return stop(t, "palm");
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
        if (label == PEACE || label == TEACH) {
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
