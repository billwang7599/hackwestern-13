// TARS pet mode, C++17, header-only. Line-for-line port of iter3-pet/vision/common/pet.py
// (same states, rules, command text and RNG). tests/test_parity.py runs both on
// the same sessions and diffs every line. Change one, change the other (and
// iter3-pet/vision/GESTURES.md section 8). Safety notes: see the top of pet.py.
#pragma once

#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "gesture.hpp"

namespace tars {
namespace pet {

inline const char *IDLE = "idle", *WATCH = "watch", *COME = "come", *STAY = "stay", *GREET = "greet",
                  *HAPPY = "happy", *SAD = "sad", *TRICK = "trick", *LOOK = "look", *SLEEP = "sleep";

inline bool is_body(const std::string &s) {
    return s == GREET || s == HAPPY || s == SAD || s == TRICK || s == LOOK;
}
inline bool is_stream(const std::string &s) { return s == WATCH || s == COME; }

struct Rng {
    unsigned long long s;
    explicit Rng(long long seed) : s((unsigned long long)(seed % 2147483648LL)) {}
    double uniform() {
        s = (s * 1103515245ULL + 12345ULL) % 2147483648ULL;
        return (double)s / 2147483648.0;
    }
};

class Pet {
public:
    Pet(const Config &cfg, Controller &ctl)
        : c(cfg), ctl(ctl), energy(cfg["pet_energy_start"]), rng((long long)cfg["pet_seed"]) {
        peace_reset(false);
        wave_reset();
    }

    const Config &c;
    Controller &ctl;
    bool active = false;
    std::string state = IDLE;
    double state_t = 0;
    double energy;
    std::optional<double> energy_t;
    Rng rng;
    std::vector<std::string> events;
    double last_hand_t = 0, next_look = 0;
    std::string fired;  // label whose reaction fired; re-armed by any other label
    double react_until = -1e18;
    std::optional<double> zero_since;
    bool peace_armed = false;
    std::optional<double> peace_since;
    int peace_n = 0, peace_gap = 0;
    std::optional<double> w_ext;
    int w_dir = 0;
    std::vector<double> w_times;
    std::optional<double> waved_t;

    double speed() const {
        double s = c["pet_speed_min"] + (c["pet_speed_max"] - c["pet_speed_min"]) * energy;
        return std::fmin(s, c["pet_max_speed"]);
    }

    // ---- mode enter / leave ----
    std::vector<Cmd> sync(double t) {
        if (ctl.mode == PET_MODE && !active) return enter(t);
        if (ctl.mode != PET_MODE && active) {
            active = false;
            set(t, IDLE);
            events.push_back("leave");
        }
        return {};
    }

    // ---- per frame ----
    std::vector<Cmd> on_frame(double t, const std::string &label, std::optional<Feat> feat) {
        decay(t);
        auto out = ctl.on_frame(t, label, feat);  // safety floor first: palm S, lost S
        reconcile(t);
        if (label != fired) fired.clear();
        if (label != NO_HAND) {
            last_hand_t = t;
            next_look = t + c["pet_bored_ms"];
        }

        // open palm: stop (done by ctl), stay, watch for a wave
        if (label == PALM) {
            peace_reset(true);
            if (state == SLEEP) {
                events.push_back("wake palm");
                append(out, fx_cmds("ok", 0, "pet_wake"));
            }
            if (state != STAY) set(t, STAY);
            if (ctl.f_active) ctl.f_tgt_l = ctl.f_tgt_r = 0.0;
            if (feat) wave(t, feat->cx);
            if (waved_t && t - *waved_t > c["pet_wave_window_ms"]) waved_t.reset();
            return out;
        }
        wave_reset();
        if (waved_t) {  // palm dropped after a wave: greet
            bool fresh = t - *waved_t <= c["pet_wave_window_ms"];
            waved_t.reset();
            if (fresh) {  // (from sleep: the palm already woke it)
                append(out, greet(t));
                return out;
            }
        }

        if (state == SLEEP) return out;  // only palm / wave wake it
        if (state == STAY) set(t, IDLE);

        // peace: flash = trick, hold pet_exit_ms = leave pet mode
        if (label == PEACE) {
            if (peace_armed) {
                if (!peace_since) peace_since = t;
                peace_n++;
                peace_gap = 0;
                if (t - *peace_since >= c["pet_exit_ms"]) {
                    append(out, leave(t));
                    return out;
                }
            }
            if (ctl.f_active) ctl.f_tgt_l = ctl.f_tgt_r = 0.0;  // hold still
            return out;
        }
        if (!peace_armed) {
            peace_reset(true);
        } else if (peace_since) {
            peace_gap++;
            if (peace_gap > c["pet_gap_frames"]) {
                bool fire = peace_n >= c["pet_trick_frames"];
                peace_reset(true);
                if (fire) {
                    append(out, trick(t, TRICK, 1));
                    return out;
                }
            }
        }

        if (label == NO_HAND || is_body(state)) return out;
        if (label == TEACH) {  // hold still (the Brain may start teaching)
            if (ctl.f_active) ctl.f_tgt_l = ctl.f_tgt_r = 0.0;
            return out;
        }

        // reactions: held stable_frames (fist: pet_sleep_frames), once per hold
        int n = ctl.cand == label ? ctl.cand_count : 0;
        bool ready = label != fired && t >= react_until;
        if (ready && n >= c["stable_frames"]) {
            if (label == THUMBS_UP) {
                fired = label;
                append(out, happy(t));
                return out;
            }
            if (label == THUMBS_DOWN) {
                fired = label;
                append(out, sad(t));
                return out;
            }
            if (label == POINT_UP && state != COME) {
                fired = label;
                events.push_back("come");
                stream(t, COME);
            }
        }
        if (ready && label == FIST && n >= c["pet_sleep_frames"]) {
            fired = label;
            append(out, sleep(t, "fist"));
            return out;
        }

        // continuous: come here / watch the hand
        if (!feat) return out;
        if (state == COME) {
            if (feat->size >= c["pet_come_size"] - c["follow_size_deadband"]) {
                boost(c["pet_come_boost"]);
                set(t, IDLE);
                events.push_back("arrived");
                append(out, ctl.stop_now(t, "pet_arrived"));
                append(out, fx_cmds("ok", 1, "pet_arrived"));
                return out;
            }
            targets(*feat, true, ctl.f_tgt_l, ctl.f_tgt_r);
            return out;
        }
        double tl, tr;
        targets(*feat, false, tl, tr);
        if (state == IDLE && !(tl == 0.0 && tr == 0.0)) stream(t, WATCH);
        if (state == WATCH) {
            ctl.f_tgt_l = tl;
            ctl.f_tgt_r = tr;
        }
        return out;
    }

    std::vector<Cmd> tick(double t) {
        if (!active) return {};
        decay(t);
        reconcile(t);
        if (state == SLEEP) return {};
        if (!ctl.last_frame_t || t - *ctl.last_frame_t >= c["frame_timeout_ms"]) return {};  // camera stalled
        if (state == COME && t - state_t >= c["pet_come_max_ms"]) {
            set(t, IDLE);
            events.push_back("come limit");
            return ctl.stop_now(t, "pet_come_limit");
        }
        if (state == WATCH) {
            if (ctl.f_tgt_l == 0.0 && ctl.f_tgt_r == 0.0 && ctl.f_cur_l == 0.0 && ctl.f_cur_r == 0.0) {
                if (!zero_since) zero_since = t;
                else if (t - *zero_since >= c["pet_settle_ms"]) {
                    set(t, IDLE);
                    return ctl.stop_now(t, "pet_settle");
                }
            } else {
                zero_since.reset();
            }
        }
        if (state == IDLE && !ctl.f_active && !ctl.has_move) {
            if (t - last_hand_t >= c["pet_sleep_after_ms"]) return sleep(t, "bored");
            if (t >= next_look) {
                next_look = t + c["pet_look_every_ms"];
                if (rng.uniform() < 0.3 + 0.7 * energy) return look(t);
            }
        }
        return {};
    }

private:
    static void append(std::vector<Cmd> &a, const std::vector<Cmd> &b) { a.insert(a.end(), b.begin(), b.end()); }
    static double clamp(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }

    void peace_reset(bool armed) {
        peace_armed = armed;
        peace_since.reset();
        peace_n = 0;
        peace_gap = 0;
    }
    void wave_reset() {
        w_ext.reset();
        w_dir = 0;
        w_times.clear();
    }
    void set(double t, const std::string &s) {
        state = s;
        state_t = t;
        zero_since.reset();
    }
    void boost(double d) { energy = clamp(energy + d, 0.0, 1.0); }
    void decay(double t) {
        if (energy_t && t > *energy_t) {
            double step = c["pet_energy_decay_per_s"] * (t - *energy_t) / 1000.0;
            energy = toward(energy, c["pet_energy_rest"], step);
        }
        if (!energy_t || t > *energy_t) energy_t = t;
    }

    std::vector<Seg> fx(const std::string &led, int beep) const {
        std::vector<Seg> v;
        if (c["feedback_led_beep"] < 0.5) return v;
        if (!led.empty()) v.push_back(segX("LED " + led, 0));
        if (beep) v.push_back(segX("BEEP " + std::to_string(beep), 0));
        return v;
    }
    std::vector<Cmd> fx_cmds(const std::string &led, int beep, const std::string &reason) const {
        std::vector<Cmd> out;
        for (auto &s : fx(led, beep)) out.push_back(Cmd{s.text, reason});
        return out;
    }
    std::vector<Seg> bound(const std::vector<Seg> &segs, bool long_ok) const {
        double mx = c["pet_max_speed"], ms_max = c["pet_max_seg_ms"];
        std::vector<Seg> out;
        for (auto s : segs) {
            if (s.d) {
                if (!long_ok) s.ms = std::fmin(s.ms, ms_max);
                s.l = clamp(s.l, -mx, mx);
                s.r = clamp(s.r, -mx, mx);
            }
            out.push_back(s);
        }
        return out;
    }
    std::vector<Cmd> play(double t, const std::string &st, const std::vector<Seg> &segs, bool long_ok = false) {
        set(t, st);
        events.push_back(st);
        return ctl.play(t, "pet_" + st, bound(segs, long_ok));
    }
    void stream(double t, const std::string &st) {
        set(t, st);
        if (!ctl.f_active) {
            ctl.f_active = true;
            ctl.last_send = t - c["pet_send_ms"];
        }
    }
    std::vector<Cmd> halt(double t, const std::string &reason) {
        if (ctl.need_stop || ctl.has_move || ctl.f_active) return ctl.stop_now(t, reason);
        return {};
    }
    static std::vector<Seg> cat(std::vector<Seg> a, const std::vector<Seg> &b) {
        a.insert(a.end(), b.begin(), b.end());
        return a;
    }

    std::vector<Cmd> enter(double t) {
        active = true;
        set(t, IDLE);
        energy = c["pet_energy_start"];
        energy_t = t;
        last_hand_t = t;
        next_look = t + c["pet_bored_ms"];
        fired.clear();
        react_until = -1e18;
        peace_reset(false);
        wave_reset();
        waved_t.reset();
        events.push_back("enter");
        auto f = fx("ok", 2);
        if (f.empty()) return {};
        return ctl.play(t, "pet_hello", f);
    }

    std::vector<Cmd> leave(double t) {
        auto out = ctl.stop_now(t, "mode_gesture");
        ctl.mode = GESTURE_MODE;
        ctl.toggle_armed = false;
        ctl.mode_cooldown_until = t + c["mode_cooldown_ms"];
        ctl.cooldown_until = t + c["cooldown_ms"];
        ctl.fired_intent.clear();
        ctl.armed = true;
        active = false;
        set(t, IDLE);
        events.push_back("leave peace");
        append(out, fx_cmds("off", 0, "mode_gesture"));
        return out;
    }

    std::vector<Cmd> greet(double t) {
        boost(c["pet_greet_boost"]);
        double s = speed(), w = c["pet_body_ms"];
        return play(t, GREET, cat(fx("ok", 2), {segD(-s, s, w), segD(s, -s, 2 * w), segD(-s, s, 2 * w), segD(s, -s, w)}));
    }
    std::vector<Cmd> trick(double t, const std::string &st, int beep) {
        double ts = c["turn_speed"];
        return play(t, st, cat(fx("ok", beep), {segD(-ts, ts, c["turn_ms"] * c["spin_turns"])}), true);
    }
    std::vector<Cmd> happy(double t) {
        boost(c["pet_happy_boost"]);
        if (energy >= c["pet_excited"]) return trick(t, HAPPY, 2);  // zoomies
        double s = speed(), w = c["pet_happy_ms"];
        return play(t, HAPPY, cat(fx("ok", 1), {segD(s, -s, w), segD(-s, s, w)}));
    }
    std::vector<Cmd> sad(double t) {
        boost(-c["pet_sad_drop"]);
        double s = speed();
        return play(t, SAD, cat(fx("no", 0), {segD(-s, -s, c["pet_sad_ms"])}));
    }
    std::vector<Cmd> look(double t) {
        double s = speed(), w = c["pet_look_ms"];
        if (rng.uniform() < 0.5) s = -s;  // look left first
        return play(t, LOOK, {segD(s, -s, w), segD(0.0, 0.0, w), segD(-s, s, w)});
    }
    std::vector<Cmd> sleep(double t, const std::string &why) {
        auto out = halt(t, "pet_sleep");
        set(t, SLEEP);
        events.push_back("sleep " + why);
        append(out, fx_cmds("off", 0, "pet_sleep"));
        return out;
    }

    double turn(double cx) const {
        auto lr = follow_targets(cx, c["follow_target_size"], c);
        return (lr.first - lr.second) / 2.0;
    }
    void targets(const Feat &f, bool come, double &l, double &r) const {
        double k = speed() / c["pet_speed_max"];
        double tn = turn(f.cx) * k, fwd = 0.0;
        if (come) {
            double e = c["pet_come_size"] - f.size, sdb = c["follow_size_deadband"];
            if (e > sdb) fwd = clamp(c["follow_kp_fwd"] * (e - sdb), 0.0, c["pet_come_max_fwd"]) * k;
        }
        double mx = c["pet_max_speed"];
        l = clamp(fwd + tn, -mx, mx);
        r = clamp(fwd - tn, -mx, mx);
    }

    void reconcile(double t) {
        if (is_stream(state) && !ctl.f_active) set(t, IDLE);
        else if (is_body(state) && !ctl.has_move) {
            if (state != LOOK) next_look = std::fmax(next_look, t + c["pet_bored_ms"]);  // boredom starts over
            set(t, IDLE);
            react_until = t + c["cooldown_ms"];
        }
    }

    void wave(double t, double x) {
        double amp = c["pet_wave_amp"];
        bool rev = false;
        if (!w_ext) {
            w_ext = x;
        } else if (w_dir == 0) {
            if (std::fabs(x - *w_ext) >= amp) {
                w_dir = x > *w_ext ? 1 : -1;
                w_ext = x;
            }
        } else if (w_dir == 1) {
            if (x > *w_ext) w_ext = x;
            else if (*w_ext - x >= amp) { w_dir = -1; w_ext = x; rev = true; }
        } else {
            if (x < *w_ext) w_ext = x;
            else if (x - *w_ext >= amp) { w_dir = 1; w_ext = x; rev = true; }
        }
        if (rev) {
            double win = c["pet_wave_window_ms"];
            std::vector<double> keep;
            for (double k : w_times)
                if (t - k <= win) keep.push_back(k);
            keep.push_back(t);
            w_times = keep;
            if ((double)w_times.size() >= c["pet_wave_swings"]) {
                if (!waved_t) events.push_back("wave");
                waved_t = t;
            }
        }
    }
};

}  // namespace pet
}  // namespace tars
