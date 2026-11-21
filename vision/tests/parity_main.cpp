// Parity driver for vision/common/gesture.hpp (built + run by test_parity.py).
// stdin script, one command per line:
//   C <path/to/gesture.conf>
//   F <t_ms> <aspect> <n> x0 y0 z0 ... x(n-1) y(n-1) z(n-1)     (n = 0: no hand)
//   T <t_ms>
// stdout, per F: "L <label> <mode>" then "> <text>|<reason>" per command;
//         per T: "> <text>|<reason>" per command.
#include <cstdio>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "../common/gesture.hpp"

int main() {
    std::unique_ptr<tars::Controller> ctl;
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream in(line);
        std::string op;
        in >> op;
        if (op == "C") {
            std::string path;
            std::getline(in >> std::ws, path);
            ctl = std::make_unique<tars::Controller>(tars::load_config(path));
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
            const tars::Landmarks *p = n ? &lm : nullptr;
            std::string label = tars::classify(p, aspect, ctl->cfg());
            auto feat = tars::hand_features(p, aspect);
            auto cmds = ctl->on_frame(t, label, feat);
            std::printf("L %s %s\n", label.c_str(), ctl->mode.c_str());
            for (auto &c : cmds) std::printf("> %s|%s\n", c.text.c_str(), c.reason.c_str());
        } else if (op == "T") {
            std::string ts;
            in >> ts;
            for (auto &c : ctl->tick(std::stod(ts)))
                std::printf("> %s|%s\n", c.text.c_str(), c.reason.c_str());
        }
    }
    return 0;
}
