// Telemetry only: intentionally has no connection to the motor server.
#include <arpa/inet.h>
#include <sys/socket.h>
#include <poll.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char** argv) {
    char* end = nullptr;
    long port = argc > 1 ? std::strtol(argv[1], &end, 10) : 5556;
    if (argc > 2 || port < 1 || port > 65535 || (end && *end)) {
        std::cerr << "Usage: imu_receiver [port 1..65535]\n"; return 1;
    }
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { perror("socket"); return 1; }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<unsigned short>(port));
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("bind"); close(fd); return 1;
    }
    std::cout << "C++ receiver running; UDP " << port << "; raw counts; no motor control" << std::endl;
    using Clock = std::chrono::steady_clock;
    auto last = Clock::now(), display = last;
    bool stale = false;
    unsigned long long count = 0;
    for (;;) {
        pollfd event{fd, POLLIN, 0};
        int ready = poll(&event, 1, 50);
        if (ready < 0) { if (errno == EINTR) continue; perror("poll"); break; }
        if (ready > 0 && (event.revents & POLLIN)) {
            char buffer[512];
            auto n = recvfrom(fd, buffer, sizeof(buffer), 0, nullptr, nullptr);
            if (n < 0) { perror("recvfrom"); break; }
            std::istringstream input(std::string(buffer, static_cast<size_t>(n)));
            std::string magic, extra;
            unsigned long long seq, stamp;
            int timer, index, v[6];
            bool valid = bool(input >> magic >> seq >> stamp >> timer >> index);
            for (int& x : v) valid = valid && bool(input >> x) && x >= -32768 && x <= 32767;
            if (!valid || magic != "IMU1" || timer < 0 || timer > 255 || index < 0 || index > 2 || (input >> extra)) continue;
            last = Clock::now();
            if (stale) std::cout << "STREAM RESUMED" << std::endl;
            stale = false;
            ++count;
            if (count == 1 || last - display >= std::chrono::milliseconds(250)) {
                std::cout << "samples=" << count << " seq=" << seq << " accel=" << v[0] << ',' << v[1] << ',' << v[2]
                          << " gyro=" << v[3] << ',' << v[4] << ',' << v[5] << std::endl;
                display = last;
            }
        }
        if (!stale && Clock::now() - last > std::chrono::milliseconds(250)) {
            std::cout << "STALE: no valid IMU datagram for 250 ms" << std::endl;
            stale = true;
        }
    }
    close(fd);
    return 1;
}
