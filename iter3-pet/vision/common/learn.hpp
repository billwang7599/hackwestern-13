// Teach TARS a new hand pose on the spot. C++17, header-only, no dependencies.
// Line-for-line port of iter3-pet/vision/common/learn.py (features, few-shot k-NN, the
// hands-only teach state machine, JSON save/load). tests/test_parity.py runs
// both on the same sessions and diffs every line. Change one, change the other.
#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "gesture.hpp"
#include "pet.hpp"

namespace tars {
namespace learn {

constexpr int N_FEAT = 57;
using Feat = std::vector<double>;

inline const int CHAINS[5][5] = {{0, 1, 2, 3, 4}, {0, 5, 6, 7, 8}, {0, 9, 10, 11, 12},
                                 {0, 13, 14, 15, 16}, {0, 17, 18, 19, 20}};

// 57 numbers invariant to position, size, in-plane rotation and handedness.
inline std::optional<Feat> features(const Landmarks *lmp, double aspect, double angle_weight) {
    if (!lmp || lmp->size() < 21) return std::nullopt;
    const Landmarks &lm = *lmp;
    double wx = lm[0].x * aspect, wy = lm[0].y;
    double mx = lm[9].x * aspect - wx, my = lm[9].y - wy;
    double s = std::sqrt(mx * mx + my * my);
    if (s < 1e-6) return std::nullopt;
    double c = -my / s, sn = -mx / s;
    double xs[21], ys[21];
    for (int i = 0; i < 21; i++) {
        double x = lm[i].x * aspect - wx, y = lm[i].y - wy;
        xs[i] = (c * x - sn * y) / s;
        ys[i] = (sn * x + c * y) / s;
    }
    if (xs[5] > xs[17])
        for (double &x : xs) x = -x;
    Feat out;
    out.reserve(N_FEAT);
    for (int i = 0; i < 21; i++) {
        out.push_back(xs[i]);
        out.push_back(ys[i]);
    }
    for (auto &ch : CHAINS)
        for (int j = 1; j < 4; j++) {
            double ax = xs[ch[j]] - xs[ch[j - 1]], ay = ys[ch[j]] - ys[ch[j - 1]];
            double bx = xs[ch[j + 1]] - xs[ch[j]], by = ys[ch[j + 1]] - ys[ch[j]];
            out.push_back(std::atan2(std::fabs(ax * by - ay * bx), ax * bx + ay * by) * angle_weight);
        }
    return out;
}

inline double dist(const Feat &a, const Feat &b) {
    double s = 0.0;
    for (int i = 0; i < N_FEAT; i++) {
        double d = a[i] - b[i];
        s += d * d;
    }
    return std::sqrt(s);
}

struct Lcg {
    unsigned long long s;
    explicit Lcg(long long seed) : s((unsigned long long)(seed % 2147483648LL)) {}
    double uniform() {
        s = (s * 1103515245ULL + 12345ULL) % 2147483648ULL;
        return (double)s / 2147483648.0;
    }
};

inline double kmean(std::vector<double> ds, int k) {
    std::sort(ds.begin(), ds.end());
    size_t n = std::min(ds.size(), (size_t)std::max(1, k));
    double s = 0.0;
    for (size_t i = 0; i < n; i++) s += ds[i];
    return s / (double)n;
}

// ---- few-shot classifier -----------------------------------------------------
struct PoseClass {
    std::string name, action;
    std::vector<Feat> raw, examples;
    long long seed = 0;
    std::vector<Seg> custom;
    double thr = 0;

    PoseClass(std::string n, std::string a, std::vector<Feat> r, long long sd, const Config &cfg,
              std::vector<Seg> cu = {}, std::optional<double> th = std::nullopt)
        : name(std::move(n)), action(std::move(a)), raw(std::move(r)), seed(sd), custom(std::move(cu)) {
        double j = cfg["learn_jitter"];
        Lcg rng(seed);
        examples = raw;
        int aug = (int)cfg["learn_augment"];
        for (auto &f : raw)
            for (int q = 0; q < aug; q++) {
                Feat e(f.size());
                for (size_t i = 0; i < f.size(); i++) e[i] = f[i] + (2.0 * rng.uniform() - 1.0) * j;
                examples.push_back(std::move(e));
            }
        thr = th ? *th : threshold(cfg);
    }

    double threshold(const Config &cfg) const {
        int k = (int)cfg["learn_k"];
        size_t n = raw.size();
        std::vector<double> loo;
        for (size_t i = 0; i < n; i++) {
            std::vector<double> ds;
            for (size_t j = 0; j < n; j++)
                if (j != i) ds.push_back(dist(raw[i], raw[j]));
            if (!ds.empty()) loo.push_back(kmean(ds, k));
        }
        double m = 0, var = 0;
        if (!loo.empty()) {
            for (double d : loo) m += d;
            m /= (double)loo.size();
            for (double d : loo) var += (d - m) * (d - m);
            var /= (double)loo.size();
        }
        double t = cfg["learn_thr_mult"] * (m + 2.0 * std::sqrt(var));
        return std::min(std::max(t, cfg["learn_thr_min"]), cfg["learn_thr_max"]);
    }

    double score(const Feat &f, int k) const {
        std::vector<double> ds;
        ds.reserve(examples.size());
        for (auto &e : examples) ds.push_back(dist(f, e));
        return kmean(ds, k);
    }
};

struct Match {
    std::string name;   // accepted name, "" = none
    std::string best;   // nearest class, "" = no classes
    double ratio = 1e18, conf = 0.0;
};

// ---- tiny JSON (just what learned.json needs) ---------------------------------
struct Json {
    enum T { NUL, NUM, STR, ARR, OBJ, BOOL } t = NUL;
    double num = 0;
    std::string str;
    std::vector<Json> arr;
    std::vector<std::pair<std::string, Json>> obj;
    const Json *get(const std::string &k) const {
        for (auto &kv : obj)
            if (kv.first == k) return &kv.second;
        return nullptr;
    }
};

struct JsonParser {
    const std::string &s;
    size_t i = 0;
    explicit JsonParser(const std::string &src) : s(src) {}
    void ws() { while (i < s.size() && std::isspace((unsigned char)s[i])) i++; }
    [[noreturn]] void fail(const char *why) { throw std::runtime_error(std::string("json: ") + why); }
    Json parse() {
        ws();
        if (i >= s.size()) fail("unexpected end");
        Json v;
        char ch = s[i];
        if (ch == '{') {
            v.t = Json::OBJ;
            i++;
            ws();
            if (s[i] == '}') { i++; return v; }
            while (true) {
                ws();
                Json k = parse();
                if (k.t != Json::STR) fail("key");
                ws();
                if (s[i++] != ':') fail("':'");
                v.obj.emplace_back(k.str, parse());
                ws();
                if (s[i] == ',') { i++; continue; }
                if (s[i] == '}') { i++; return v; }
                fail("',' or '}'");
            }
        }
        if (ch == '[') {
            v.t = Json::ARR;
            i++;
            ws();
            if (s[i] == ']') { i++; return v; }
            while (true) {
                v.arr.push_back(parse());
                ws();
                if (s[i] == ',') { i++; continue; }
                if (s[i] == ']') { i++; return v; }
                fail("',' or ']'");
            }
        }
        if (ch == '"') {
            v.t = Json::STR;
            i++;
            while (i < s.size() && s[i] != '"') {
                if (s[i] == '\\' && i + 1 < s.size()) i++;
                v.str += s[i++];
            }
            i++;
            return v;
        }
        if (s.compare(i, 4, "true") == 0) { i += 4; v.t = Json::BOOL; v.num = 1; return v; }
        if (s.compare(i, 5, "false") == 0) { i += 5; v.t = Json::BOOL; return v; }
        if (s.compare(i, 4, "null") == 0) { i += 4; return v; }
        char *end = nullptr;
        v.num = std::strtod(s.c_str() + i, &end);
        if (end == s.c_str() + i) fail("value");
        i = (size_t)(end - s.c_str());
        v.t = Json::NUM;
        return v;
    }
};

inline std::string jnum(double x) {
    char b[40];
    std::snprintf(b, sizeof b, "%.17g", x);
    std::string s = b;
    if (s.find_first_of(".eEn") == std::string::npos) s += ".0";
    return s;
}

class Learner {
public:
    explicit Learner(const Config &cfg) : cfg_(cfg) {}
    std::vector<PoseClass> classes;
    long long next_seed = 1;

    Match match(const Feat &f) const {
        const PoseClass *best = nullptr;
        double br = 1e18;
        int k = (int)cfg_["learn_k"];
        for (auto &c : classes) {
            double r = c.score(f, k) / c.thr;
            if (r < br) { best = &c; br = r; }
        }
        Match m;
        if (!best) return m;
        m.best = best->name;
        m.ratio = br;
        m.conf = 1.0 / (1.0 + br * br);
        if (br <= 1.0) m.name = best->name;
        return m;
    }

    std::string similar_to(const std::vector<Feat> &feats) const {
        int k = (int)cfg_["learn_k"];
        for (auto &c : classes) {
            int hit = 0;
            for (auto &f : feats)
                if (c.score(f, k) <= c.thr) hit++;
            if (hit >= cfg_["learn_similar_frac"] * (double)feats.size()) return c.name;
        }
        return "";
    }

    std::string free_name() const {
        for (int i = 1;; i++) {
            std::string n = "pose" + std::to_string(i);
            bool used = false;
            for (auto &c : classes) used = used || c.name == n;
            if (!used) return n;
        }
    }

    PoseClass &add(const std::string &name, const std::string &action, const std::vector<Feat> &raw,
                   const std::vector<Seg> &custom = {}) {
        classes.emplace_back(name, action, raw, next_seed, cfg_, custom);
        next_seed++;
        return classes.back();
    }

    int forget_all() {
        int n = (int)classes.size();
        classes.clear();
        return n;
    }

    std::string to_json() const {
        std::string o = "{\"version\": 1, \"next_seed\": " + std::to_string(next_seed) + ", \"classes\": [";
        for (size_t ci = 0; ci < classes.size(); ci++) {
            auto &c = classes[ci];
            if (ci) o += ", ";
            o += "{\"name\": \"" + c.name + "\", \"action\": \"" + c.action + "\", \"seed\": " +
                 std::to_string(c.seed) + ", \"thr\": " + jnum(c.thr) + ", \"custom\": [";
            for (size_t si = 0; si < c.custom.size(); si++) {
                auto &s = c.custom[si];
                if (si) o += ", ";
                if (s.d) o += "[\"D\", " + jnum(s.l) + ", " + jnum(s.r) + ", " + jnum(s.ms) + "]";
                else o += "[\"X\", \"" + s.text + "\", 0.0, " + jnum(s.ms) + "]";
            }
            o += "], \"examples\": [";
            for (size_t ei = 0; ei < c.raw.size(); ei++) {
                if (ei) o += ", ";
                o += "[";
                for (size_t k = 0; k < c.raw[ei].size(); k++) o += (k ? ", " : "") + jnum(c.raw[ei][k]);
                o += "]";
            }
            o += "]}";
        }
        return o + "]}";
    }

    void from_json(const Json &d) {
        classes.clear();
        const Json *cs = d.get("classes");
        if (cs)
            for (auto &c : cs->arr) {
                std::vector<Seg> custom;
                if (const Json *cu = c.get("custom"))
                    for (auto &s : cu->arr) {
                        if (s.arr.size() < 4) continue;
                        if (s.arr[0].str == "D") custom.push_back(segD(s.arr[1].num, s.arr[2].num, s.arr[3].num));
                        else custom.push_back(segX(s.arr[1].str, s.arr[3].num));
                    }
                std::vector<Feat> raw;
                for (auto &e : c.get("examples")->arr) {
                    Feat f;
                    for (auto &x : e.arr) f.push_back(x.num);
                    if (f.size() != (size_t)N_FEAT) throw std::runtime_error("json: bad example length");
                    raw.push_back(std::move(f));
                }
                classes.emplace_back(c.get("name")->str, c.get("action")->str, raw,
                                     (long long)c.get("seed")->num, cfg_, custom, c.get("thr")->num);
            }
        const Json *ns = d.get("next_seed");
        next_seed = ns ? (long long)ns->num : (long long)classes.size() + 1;
    }

private:
    const Config &cfg_;
};

// ---- teach flow + combined pipeline (same as learn.py Brain) ---------------------
inline const char *IDLE = "idle", *COUNTDOWN = "countdown", *RECORD = "record", *ASSIGN = "assign",
                  *CONFIRM = "confirm";

inline std::string assign_action(const std::string &rule) {
    if (rule == POINT_RIGHT) return TURN_RIGHT;
    if (rule == POINT_LEFT) return TURN_LEFT;
    if (rule == THUMBS_UP || rule == POINT_UP) return FORWARD;
    if (rule == FIST || rule == THUMBS_DOWN || rule == POINT_DOWN) return BACK;
    if (rule == PEACE) return "spin";
    return "";
}

inline const std::vector<std::string> &all_labels() {
    static const std::vector<std::string> l = {NO_HAND,     UNKNOWN,    PALM,       FIST,
                                               THUMBS_UP,   THUMBS_DOWN, POINT_UP,  POINT_DOWN,
                                               POINT_LEFT,  POINT_RIGHT, PEACE,     TEACH};
    return l;
}

class Brain {
public:
    explicit Brain(Config cfg) : ctl(std::move(cfg)), learner(ctl.cfg()), pet(ctl.cfg(), ctl) {
        if (ctl.cfg()["pet_start"] >= 0.5) ctl.mode = PET_MODE;
    }

    Controller ctl;
    Learner learner;
    pet::Pet pet;  // pet mode (GESTURES.md section 8)
    std::string state = IDLE;
    double state_t = 0;
    std::vector<std::string> events;
    bool dirty = false;
    std::string last_rule = NO_HAND, label = NO_HAND;
    bool has_match = false;
    Match match;
    std::optional<double> teach_since;
    int gap = 0;
    bool released = true;
    int nudges = 0, palm_count = 0;
    std::vector<Feat> rec;
    std::vector<std::string> rec_labels;
    std::string cand;
    int cand_count = 0;
    std::string hold_off;  // "" = none
    double cooldown_until = -1e18;

    const Config &cfg() const { return ctl.cfg(); }

    void sync() {
        ctl.learned.clear();
        for (auto &c : learner.classes) ctl.learned[c.name] = {c.action, c.custom};
    }

    void load_json_text(const std::string &text) {
        JsonParser p(text);
        learner.from_json(p.parse());
        sync();
    }

    int load(const std::string &path) {
        std::ifstream f(path);
        if (!f) return 0;
        std::stringstream ss;
        ss << f.rdbuf();
        load_json_text(ss.str());
        return (int)learner.classes.size();
    }

    void save(const std::string &path) {
        std::string tmp = path + ".tmp";
        {
            std::ofstream f(tmp);
            if (!f) throw std::runtime_error("cannot write " + tmp);
            f << learner.to_json();
        }
        if (std::rename(tmp.c_str(), path.c_str()) != 0) throw std::runtime_error("cannot rename " + tmp);
        dirty = false;
    }

    // -> (rule label, combined label)
    std::pair<std::string, std::string> classify(const Landmarks *lm, double aspect) {
        std::string rule = tars::classify(lm, aspect, cfg());
        has_match = false;
        if (rule != UNKNOWN || learner.classes.empty()) return {rule, rule};
        auto f = features(lm, aspect, cfg()["learn_angle_weight"]);
        if (!f) return {rule, rule};
        match = learner.match(*f);
        has_match = true;
        return {rule, match.name.empty() ? std::string(UNKNOWN) : match.name};
    }

    // tars_motor refused a D (S from another client, watchdog, tip, bump, guard):
    // drop the move/stream so it never restarts by itself. Sends nothing.
    void server_stop(double t) {
        if (ctl.need_stop || ctl.has_move || ctl.f_active) ctl.stop_now(t, "server_stop");
    }

    std::vector<Cmd> button(double t, const std::string &kind) {
        if (kind == "forget") {
            std::vector<Cmd> out;
            if (state != IDLE) out = ctl.stop_now(t, "teach");
            append(out, forget(t, "button"));
            return out;
        }
        if (kind == "teach") {
            if (state == IDLE) return start_teach(t, "button");
            if (state != CONFIRM) {
                events.push_back("cancel button");
                auto out = ctl.stop_now(t, "teach_cancel");
                append(out, ctl.play(t, "teach_cancel", fb("cancel")));
                end(t);
                return out;
            }
        }
        return {};
    }

    std::vector<Cmd> on_frame(double t, const Landmarks *lm, double aspect) {
        const Config &c = cfg();
        auto rl = classify(lm, aspect);
        std::string rule = rl.first, lab = rl.second;
        last_rule = rule;
        auto feat = hand_features(lm, aspect);
        if (state == IDLE) {
            label = lab;
            if (rule == TEACH) {
                if (!teach_since) teach_since = t;
                gap = 0;
            } else if (teach_since) {
                gap++;
                if (gap > c["learn_gap_frames"]) teach_since.reset();
            }
            if (teach_since && t - *teach_since >= c["teach_hold_ms"] && t >= cooldown_until)
                return start_teach(t, "gesture");
            if (!hold_off.empty()) {
                if (rule == hold_off) lab = UNKNOWN;
                else hold_off.clear();
            }
            auto out = pet.sync(t);
            if (pet.active) append(out, pet.on_frame(t, lab, feat));
            else append(out, ctl.on_frame(t, lab, feat));
            append(out, pet.sync(t));
            return out;
        }
        label = rule;
        palm_count = rule == PALM ? palm_count + 1 : 0;
        auto out = ctl.on_frame(t, rule == PALM ? PALM : UNKNOWN, std::nullopt);
        if (state != CONFIRM && palm_count >= c["learn_cancel_frames"]) {
            events.push_back("cancel palm");
            append(out, ctl.play(t, "teach_cancel", fb("cancel")));
            end(t);
            return out;
        }
        if (state == COUNTDOWN) {
            if (rule == TEACH) gap = 0;
            else if (++gap > c["learn_gap_frames"]) released = true;
        } else if (state == RECORD) {
            if (lm && rule != PALM) {
                auto f = features(lm, aspect, c["learn_angle_weight"]);
                if (f) {
                    rec.push_back(*f);
                    rec_labels.push_back(rule);
                }
                if ((double)rec.size() >= c["learn_frames"]) {
                    append(out, finish_record(t));
                    return out;
                }
            }
        } else if (state == ASSIGN) {
            std::string act = assign_action(rule);
            if (act.empty()) { cand.clear(); cand_count = 0; }
            else if (act == cand) cand_count++;
            else { cand = act; cand_count = 1; }
            if (!act.empty() && cand_count >= c["learn_assign_frames"]) {
                append(out, assign(t, act, "gesture"));
                return out;
            }
        }
        append(out, teach_tick(t));
        return out;
    }

    std::vector<Cmd> tick(double t) {
        auto out = ctl.tick(t);
        if (state != IDLE) {
            append(out, teach_tick(t));
        } else {
            append(out, pet.sync(t));
            append(out, pet.tick(t));
        }
        return out;
    }

private:
    static void append(std::vector<Cmd> &a, const std::vector<Cmd> &b) { a.insert(a.end(), b.begin(), b.end()); }

    std::vector<Seg> fb(const std::string &kind) const {
        const Config &c = cfg();
        double f = c["feedback_speed"], w = c["feedback_ms"], n = c["nudge_ms"];
        Seg L = segD(-f, f, w), R = segD(f, -f, w);
        std::vector<Seg> fx;
        if (c["feedback_led_beep"] >= 0.5) {
            static const std::map<std::string, std::pair<std::string, int>> m = {
                {"armed", {"teach", 1}}, {"nudge", {"", 1}},   {"record", {"rec", 2}},
                {"recorded", {"ok", 2}}, {"no", {"no", 3}},    {"forget", {"forget", 4}},
                {"learned", {"ok", 0}},  {"cancel", {"off", 0}}};
            auto &lb = m.at(kind);
            if (!lb.first.empty()) fx.push_back(segX("LED " + lb.first, 0));
            if (lb.second) fx.push_back(segX("BEEP " + std::to_string(lb.second), 0));
        }
        auto cat = [&](std::vector<Seg> more) {
            std::vector<Seg> v = fx;
            v.insert(v.end(), more.begin(), more.end());
            return v;
        };
        if (kind == "armed") return cat({L, R});
        if (kind == "nudge") return cat({segD(f, f, n), segD(-f, -f, n)});
        if (kind == "recorded") return cat({L, R, L, R});
        if (kind == "no") return cat({L, segD(f, -f, 2 * w), L});
        if (kind == "forget") return cat({segD(-f, -f, n), segD(f, f, n), segD(-f, -f, n), segD(f, f, n)});
        return fx;
    }

    std::vector<Cmd> start_teach(double t, const std::string &via) {
        auto out = ctl.stop_now(t, "teach");
        ctl.mode = GESTURE_MODE;
        pet.sync(t);  // teaching leaves pet mode
        state = COUNTDOWN;
        state_t = t;
        released = via == "button";
        gap = nudges = palm_count = 0;
        rec.clear();
        rec_labels.clear();
        events.push_back("teach start " + via);
        append(out, ctl.play(t, "teach_armed", fb("armed")));
        return out;
    }

    void end(double t) {
        state = IDLE;
        teach_since.reset();
        gap = 0;
        hold_off = (last_rule == NO_HAND || last_rule == UNKNOWN || last_rule == PALM) ? "" : last_rule;
        cooldown_until = t + cfg()["learn_cooldown_ms"];
    }

    std::vector<Cmd> reject(double t, const std::string &why) {
        events.push_back("reject " + why);
        state = CONFIRM;
        state_t = t;
        return ctl.play(t, "teach_no", fb("no"));
    }

    std::vector<Cmd> forget(double t, const std::string &via) {
        int n = learner.forget_all();
        sync();
        dirty = true;
        events.push_back("forget " + std::to_string(n) + " " + via);
        state = CONFIRM;
        state_t = t;
        return ctl.play(t, "teach_forget", fb("forget"));
    }

    std::vector<Cmd> finish_record(double t) {
        const Config &c = cfg();
        std::map<std::string, int> counts;
        for (auto &l : rec_labels)
            if (l != UNKNOWN && l != NO_HAND) counts[l]++;
        std::string top;
        int topn = 0;
        for (auto &l : all_labels()) {
            auto it = counts.find(l);
            if (it != counts.end() && it->second > topn) { top = l; topn = it->second; }
        }
        if (topn && topn >= c["learn_builtin_frac"] * (double)rec_labels.size())
            return reject(t, "too_similar " + top);
        if ((double)learner.classes.size() >= c["learn_max_classes"]) return reject(t, "full");
        std::string sim = learner.similar_to(rec);
        if (!sim.empty()) return reject(t, "too_similar " + sim);
        state = ASSIGN;
        state_t = t;
        cand.clear();
        cand_count = 0;
        events.push_back("recorded " + std::to_string(rec.size()));
        return ctl.play(t, "teach_recorded", fb("recorded"));
    }

    std::vector<Cmd> assign(double t, const std::string &action, const std::string &via) {
        std::string name = learner.free_name();
        learner.add(name, action, rec);
        sync();
        dirty = true;
        events.push_back("learned " + name + " " + action + " " + via);
        state = CONFIRM;
        state_t = t;
        auto segs = fb("learned");
        auto act = ctl.action_segments(action);
        segs.insert(segs.end(), act.begin(), act.end());
        return ctl.play(t, action + "@" + name, segs);
    }

    std::vector<Cmd> teach_tick(double t) {
        const Config &c = cfg();
        double el = t - state_t;
        if (state == COUNTDOWN) {
            if (el >= c["teach_countdown_ms"]) {
                if (!released) return forget(t, "gesture");
                state = RECORD;
                state_t = t;
                events.push_back("record");
                return ctl.play(t, "teach_record", fb("record"));
            }
            if (nudges < (int)std::floor(el / 1000.0) &&
                nudges < (int)std::floor(c["teach_countdown_ms"] / 1000.0) - 1) {
                nudges++;
                return ctl.play(t, "teach_nudge", fb("nudge"));
            }
        } else if (state == RECORD) {
            if (el >= c["learn_record_timeout_ms"]) return reject(t, "no_hand");
        } else if (state == ASSIGN) {
            if (el >= c["learn_assign_ms"]) return assign(t, "wiggle", "timeout");
        } else if (state == CONFIRM) {
            if (!ctl.has_move) {
                events.push_back("idle");
                end(t);
            }
        }
        return {};
    }
};

}  // namespace learn
}  // namespace tars
