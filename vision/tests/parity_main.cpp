// Parity driver for vision/common/gesture.hpp + learn.hpp (built + run by test_parity.py).
// stdin script, one command per line:
//   C <path/to/gesture.conf>
//   F <t_ms> <aspect> <n> x0 y0 z0 ... x(n-1) y(n-1) z(n-1)     (n = 0: no hand)
//   T <t_ms>
//   B <t_ms> teach|forget                                      (Uno button)
//   SAVE <path>   LOAD <path>   DB
//   BENCH <reps> <aspect> x0 y0 z0 ... (21 pts): time features + k-NN -> "BENCH <us/frame>"
// stdout, per F: "L <label> <mode> <state>" then "> <text>|<reason>" per command,
// then "E <event>" per teach event; T/B: commands + events; DB: one line per class.
#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "../common/learn.hpp"

using tars::learn::Brain;

static void dump(Brain &b, const std::vector<tars::Cmd> &cmds) {
    for (auto &c : cmds) std::printf("> %s|%s\n", c.text.c_str(), c.reason.c_str());
    for (auto &e : b.events) std::printf("E %s\n", e.c_str());
    b.events.clear();
}

int main() {
    std::unique_ptr<Brain> b;
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream in(line);
        std::string op;
        in >> op;
        if (op == "C") {
            std::string path;
            std::getline(in >> std::ws, path);
            b = std::make_unique<Brain>(tars::load_config(path));
        } else if (op == "F") {
            std::string ts, as;
            int n;
            in >> ts >> as >> n;
            double t = std::stod(ts), aspect = std::stod(as);
            tars::Landmarks lm(n);
            for (int i = 0; i < n; i++) {
                std::string x, y, z;
                in >> x >> y >> z;
                lm[i] = {std::stod(x), std::stod(y), std::stod(z)};
            }
            auto cmds = b->on_frame(t, n ? &lm : nullptr, aspect);
            std::printf("L %s %s %s\n", b->label.c_str(), b->ctl.mode.c_str(), b->state.c_str());
            dump(*b, cmds);
        } else if (op == "T") {
            std::string ts;
            in >> ts;
            dump(*b, b->tick(std::stod(ts)));
        } else if (op == "B") {
            std::string ts, kind;
            in >> ts >> kind;
            dump(*b, b->button(std::stod(ts), kind));
        } else if (op == "SAVE") {
            std::string p;
            std::getline(in >> std::ws, p);
            b->save(p);
        } else if (op == "LOAD") {
            std::string p;
            std::getline(in >> std::ws, p);
            std::printf("LOADED %d\n", b->load(p));
        } else if (op == "BENCH") {
            int reps;
            double aspect;
            in >> reps >> aspect;
            tars::Landmarks lm(21);
            for (auto &p : lm) in >> p.x >> p.y >> p.z;
            double aw = b->cfg()["learn_angle_weight"], sink = 0;
            auto t0 = std::chrono::steady_clock::now();
            for (int i = 0; i < reps; i++) {
                lm[0].x += 1e-9;  // defeat hoisting
                auto f = tars::learn::features(&lm, aspect, aw);
                sink += b->learner.match(*f).ratio;
            }
            double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
            std::printf("BENCH %.2f %d\n", us / reps, sink > 0 ? 1 : 0);
        } else if (op == "DB") {
            for (auto &c : b->learner.classes)
                std::printf("DB %s %s %.9f %zu\n", c.name.c_str(), c.action.c_str(), c.thr, c.examples.size());
        }
    }
    return 0;
}
