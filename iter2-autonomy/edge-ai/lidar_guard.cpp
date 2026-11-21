// Independent LiDAR clearance gate. No AI dependency; native motor consumes a bounded heartbeat.
// QNX scheduling: https://qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.lib_ref/topic/t/threadctl.html
#include <arpa/inet.h>
#include <sys/socket.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sched.h>
#ifdef __QNXNTO__
#include <sys/neutrino.h>
#endif
#include <algorithm>
#include <functional>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <limits>
#include <thread>

using Clock = std::chrono::steady_clock;
using namespace std::chrono;
constexpr size_t HEADER = 44, MAX_POINTS = 1024, MAX_PACKET = HEADER + MAX_POINTS * 12;
constexpr int STALE_MS = 150;
static std::atomic<bool> stopping{false};
static_assert(std::atomic<bool>::is_always_lock_free, "Signal-safe stop flag must be lock-free");
static void on_signal(int) { stopping = 1; }
static double millis(Clock::time_point a, Clock::time_point b) {
    return duration<double, std::milli>(a - b).count();
}
static uint32_t u32(const uint8_t* p) {
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
}
static uint64_t u64(const uint8_t* p) { return (uint64_t(u32(p)) << 32) | u32(p + 4); }
static bool scalar(const uint8_t* p, float& value) {
    uint32_t bits = u32(p); std::memcpy(&value, &bits, sizeof value);
    return std::isfinite(value) && std::abs(value) <= 1000.f;
}
// Optional conservative RGB-depth veto. This socket is polled without waiting
// and can only remove a measured LiDAR permission; silence never grants one.
static uint64_t host_us() {
    timespec t{}; clock_gettime(CLOCK_MONOTONIC, &t);
    return uint64_t(t.tv_sec) * 1000000 + uint64_t(t.tv_nsec) / 1000;
}
struct Fusion {
    int fd = -1;
    uint64_t sent = 0, expires = 0, accepted = 0, invalid = 0;
    float nearest = -1;
    bool open(unsigned port) {
        if (!port) return true;
        fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd < 0) return false;
        const int flags = fcntl(fd, F_GETFL, 0);
        if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) return false;
        sockaddr_in address{}; address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(uint16_t(port));
        return bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof address) == 0;
    }
    // Preserve a stop seen in this receive batch even if a later clear follows.
    bool poll() {
        bool observed_stop = false;
        if (fd < 0) return false;
        uint8_t packet[32];
        for (int i = 0; i < 8; ++i) {
            const ssize_t length = recv(fd, packet, sizeof packet, 0);
            if (length < 0) break;
            float distance;
            if (length != 24 || std::memcmp(packet, "FUS1", 4) || !scalar(packet + 20, distance) || (distance < 0 && distance != -1)) {
                ++invalid; continue;
            }
            const uint64_t generated = u64(packet + 4), deadline = u64(packet + 12), now = host_us();
            // Both times are on this Pi. Cached camera observations must retain
            // their original expiry; short publisher leases also detect a dead
            // SceneFeed. Explicit invalidation is (expiry=0, nearest=-1).
            if (!generated || generated <= sent || generated > now || now - generated > 400000 ||
                (deadline == 0 ? distance != -1 :
                 (deadline <= now || deadline < generated || deadline - generated > 500000))) {
                ++invalid; continue;
            }
            sent = generated; expires = deadline; nearest = distance; ++accepted;
            if (deadline && distance >= 0 && distance <= .6f) observed_stop = true;
        }
        return observed_stop;
    }
    bool live(uint64_t now) const { return fd >= 0 && expires > now; }
    ~Fusion() { if (fd >= 0) close(fd); }
};
enum class Verdict { Unknown, Blocked, Clear, Caution };
struct Sample {
    bool map_supported=false;
    Clock::time_point received{};
    Verdict verdict = Verdict::Unknown;
    uint32_t seq = 0, points = 0;
    uint64_t source_us = 0, frames = 0;
    double nearest = -1;
    double backlog_ms = 0;
    std::array<unsigned, 3> floor_hits{};
};
struct Shared {
    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    Sample sample;
    std::atomic<bool> connected{false};
    std::atomic<uint64_t> invalid{0};
};
struct Scheduling {
    int policy = 0, priority = 0, error = 0, affinity_error = 0;
    uint32_t runmask = 0;
};
static Scheduling schedule(int priority, int cpu) {
    Scheduling result;
#ifdef __QNXNTO__
    sched_param requested{}; requested.sched_priority = priority;
    result.error = pthread_setschedparam(pthread_self(), SCHED_RR, &requested);
    uint64_t mask = uint64_t(1) << cpu;
    if (ThreadCtl(_NTO_TCTL_RUNMASK, reinterpret_cast<void*>(uintptr_t(mask))) == -1)
        result.affinity_error = errno;
    // GET_AND_SET with an input mask of zero queries without changing affinity.
    unsigned actual_runmask = 0;
    if (ThreadCtl(_NTO_TCTL_RUNMASK_GET_AND_SET, &actual_runmask) == 0)
        result.runmask = actual_runmask;
    else if (!result.affinity_error)
        result.affinity_error = errno;
#else
    (void) priority; (void) cpu;
    result.error = ENOTSUP; result.affinity_error = ENOTSUP;
#endif
    sched_param actual_schedule{};
    const int query_error = pthread_getschedparam(pthread_self(), &result.policy, &actual_schedule);
    if (query_error && !result.error) result.error = query_error;
    result.priority = actual_schedule.sched_priority;
    return result;
}
static bool parse(const uint8_t* p, size_t size, Sample& out) {
    if (size < HEADER || std::memcmp(p, "LDR1", 4) != 0) return false;
    const uint32_t flags = u32(p + 16), count = u32(p + 20);
    if ((flags & ~7u) != 0 || count > MAX_POINTS || size != HEADER + count * 12) return false;
    float pose[4], floor;
    for (int i = 0; i < 4; ++i) if (!scalar(p + 24 + i * 4, pose[i])) return false;
    if (!scalar(p + 40, floor)) return false;
    out = Sample{};
    out.seq = u32(p + 4); out.source_us = u64(p + 8); out.points = count;
    if (!out.source_us) return false;
    const double forward_x = std::sin(pose[3]), forward_z = std::cos(pose[3]);
    bool blocked = false;
    for (uint32_t i = 0; i < count; ++i) {
        float point[3];
        for (int j = 0; j < 3; ++j)
            if (!scalar(p + HEADER + i * 12 + j * 4, point[j])) return false;
        const double dx = point[0] - pose[0], dz = point[2] - pose[2];
        const double forward = forward_x * dx + forward_z * dz;
        const double side = forward_z * dx - forward_x * dz;
        const double height = point[1] - floor;
        if (std::abs(side) < .45 && forward > 0 && height >= .08 && height <= 1.2) {
            if (out.nearest < 0 || forward < out.nearest) out.nearest = forward;
            if (forward <= double(.6f)) blocked = true;
        }
        if (std::abs(side) < .45 && forward >= .15 && forward <= 1.5 && std::abs(height) <= .06)
            ++out.floor_hits[side < -.15 ? 0 : (side > .15 ? 2 : 1)];
    }
    // Invalid tracking/calibration can never authorize clearance, even if the
    // geometry happens to look empty. BLOCKED takes priority only when calibrated.
    if (flags != 7) { out.nearest = -1; return true; }
    if (blocked) out.verdict = Verdict::Blocked;
    else if (count >= 15 && out.floor_hits[0] >= 2 && out.floor_hits[1] >= 2 && out.floor_hits[2] >= 2)
        out.verdict = Verdict::Clear;
    else if (count > 0) out.verdict = Verdict::Caution;
    return true;
}
static bool ready(int fd, short events, Clock::time_point deadline) {
    while (!stopping) {
        auto remaining = duration_cast<milliseconds>(deadline - Clock::now()).count();
        if (remaining <= 0) return false;
        pollfd request{fd, events, 0};
        int result = poll(&request, 1, int(remaining > 50 ? 50 : remaining));
        if (result < 0 && errno != EINTR) return false;
        if (result > 0) {
            if (request.revents & events) return true;
            if (request.revents & (POLLERR | POLLHUP | POLLNVAL)) return false;
        }
    }
    return false;
}
static bool exact(int fd, uint8_t* buffer, size_t length, Clock::time_point deadline) {
    size_t offset = 0;
    while (offset < length && ready(fd, POLLIN, deadline)) {
        ssize_t n = recv(fd, buffer + offset, length - offset, 0);
        if (n == 0) return false;
        if (n < 0) { if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue; return false; }
        offset += size_t(n);
    }
    return offset == length;
}
static int connect_local(unsigned port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    sockaddr_in address{}; address.sin_family = AF_INET; address.sin_port = htons(uint16_t(port));
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    int result = connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof address);
    if (result < 0 && (errno != EINPROGRESS || !ready(fd, POLLOUT, Clock::now() + milliseconds(300)))) {
        close(fd); return -1;
    }
    int error = 0; socklen_t size = sizeof error;
    if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &size) || error) { close(fd); return -1; }
    return fd;
}
static void receiver(Shared& shared, unsigned port, int cpu) {
    const Scheduling scheduling = schedule(18, cpu);
    std::fprintf(stderr, "LiDAR receiver policy=%d priority=%d runmask=%u schedule_error=%d affinity_error=%d\n",
                 scheduling.policy, scheduling.priority, scheduling.runmask, scheduling.error, scheduling.affinity_error);
    std::array<uint8_t, MAX_PACKET> payload{};
    uint64_t accepted = 0;
    while (!stopping) {
        int fd = connect_local(port);
        if (fd < 0) { std::this_thread::sleep_for(milliseconds(100)); continue; }
        uint64_t previous_source = 0, first_source = 0;
        uint32_t previous_seq = 0;
        // World coordinates of recently measured floor in a directly clear frame.
        std::array<std::array<float,2>, MAX_POINTS> floor_map{};
        unsigned mapped = 0;
        uint64_t mapped_us = 0;
        float mapped_floor = 0;
        Clock::time_point first_receipt{};
        while (!stopping) {
            uint8_t size_bytes[4];
            auto deadline = Clock::now() + milliseconds(300);
            if (!exact(fd, size_bytes, 4, deadline)) break;
            const uint32_t size = u32(size_bytes);
            if (size < HEADER || size > MAX_PACKET) { ++shared.invalid; break; }
            if (!exact(fd, payload.data(), size, deadline)) break;
            Sample candidate;
            if (!parse(payload.data(), size, candidate) ||
                (previous_source && (candidate.source_us <= previous_source || candidate.seq <= previous_seq))) {
                ++shared.invalid; break;
            }
            candidate.received = Clock::now();
            if (!first_source) { first_source = candidate.source_us; first_receipt = candidate.received; }
            candidate.backlog_ms = std::max(0.0, millis(candidate.received, first_receipt) -
                double(candidate.source_us - first_source) / 1000.0);
            const auto* packet = payload.data();
            float x,z,yaw,floor;
            scalar(packet+24,x); scalar(packet+32,z); scalar(packet+36,yaw); scalar(packet+40,floor);
            if (u32(packet+16)!=7 || candidate.backlog_ms>100 || candidate.verdict==Verdict::Blocked)
                mapped=0;
            else if (candidate.verdict==Verdict::Clear) {
                mapped=0; mapped_us=candidate.source_us; mapped_floor=floor;
                for (unsigned i=0;i<candidate.points;i++) {
                    const auto* point=packet+HEADER+12*i;
                    float px,py,pz; scalar(point,px); scalar(point+4,py); scalar(point+8,pz);
                    if (std::abs(py-floor)<=.06) floor_map[mapped++]={px,pz};
                }
            } else if (mapped && candidate.source_us-mapped_us<=500000 && std::abs(mapped_floor-floor)<.04 && candidate.points>0) {
                std::array<bool,9> covered{};
                for (unsigned i=0;i<mapped;i++) {
                    double dx=floor_map[i][0]-x, dz=floor_map[i][1]-z;
                    double f=dx*std::sin(yaw)+dz*std::cos(yaw);
                    double side=dx*std::cos(yaw)-dz*std::sin(yaw);
                    if (f>=.15 && f<.60 && side>=-.45 && side<.45)
                        covered[std::min(2,int((f-.15)/.15))*3+std::min(2,int((side+.45)/.30))]=true;
                }
                if (std::all_of(covered.begin(),covered.end(),[](bool v){return v;})) {
                    candidate.verdict=Verdict::Clear;
                    candidate.map_supported=true;
                }
            }
            previous_source = candidate.source_us; previous_seq = candidate.seq;
            candidate.frames = ++accepted;
            pthread_mutex_lock(&shared.mutex);
            shared.sample = candidate;
            pthread_mutex_unlock(&shared.mutex);
            shared.connected.store(true, std::memory_order_release);
            const uint8_t ack = 1;
            if (send(fd, &ack, 1, 0) != 1) break;
        }
        shared.connected.store(false, std::memory_order_release);
        close(fd);
        std::this_thread::sleep_for(milliseconds(100));
    }
}
static int number(const char* value, int minimum, int maximum) {
    char* end = nullptr; long n = std::strtol(value, &end, 10);
    if (!value[0] || *end || n < minimum || n > maximum) return -1;
    return int(n);
}
int main(int argc, char** argv) {
    unsigned port = 5562, status_port = 5564, motor_port = 0, fusion_port = 0;
    int seconds = 0, cpu = 3;
    for (int i = 1; i < argc; ++i) {
        if (i + 1 >= argc) { std::fprintf(stderr, "Missing option value\n"); return 2; }
        const char* key = argv[i]; int value = number(argv[++i], 0, 65535);
        if (value < 0) return 2;
        if (!std::strcmp(key, "--port") && value) port = unsigned(value);
        else if (!std::strcmp(key, "--status-port") && value) status_port = unsigned(value);
        else if (!std::strcmp(key, "--motor-port")) motor_port = unsigned(value);
        else if (!std::strcmp(key, "--fusion-port")) fusion_port = unsigned(value);
        else if (!std::strcmp(key, "--seconds")) seconds = value;
        else if (!std::strcmp(key, "--cpu") && value < 32) cpu = value;
        else { std::fprintf(stderr, "Unknown/invalid option: %s\n", key); return 2; }
    }
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal); signal(SIGPIPE, SIG_IGN);
    int output = socket(AF_INET, SOCK_DGRAM, 0);
    if (output < 0) { std::perror("status socket"); return 1; }
    fcntl(output, F_SETFL, fcntl(output, F_GETFL, 0) | O_NONBLOCK);
    sockaddr_in target{}; target.sin_family = AF_INET; target.sin_port = htons(uint16_t(status_port));
    target.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    sockaddr_in motor_target = target; motor_target.sin_port = htons(uint16_t(motor_port));
    auto motor_gate = [&](bool permit) {
        if (!motor_port) return;
        uint8_t packet[13] = {'G','A','T','1'};
        timespec t{}; clock_gettime(CLOCK_MONOTONIC, &t);
        uint64_t stamp = uint64_t(t.tv_sec) * 1000000 + uint64_t(t.tv_nsec) / 1000;
        for (int i = 0; i < 8; ++i) packet[4+i] = uint8_t(stamp >> (56 - 8*i));
        packet[12] = permit ? 1 : 0;
        sendto(output, packet, sizeof packet, 0, reinterpret_cast<sockaddr*>(&motor_target), sizeof motor_target);
    };
    Fusion fusion;
    if (!fusion.open(fusion_port)) { std::perror("fusion UDP bind"); close(output); return 1; }
    Shared shared;
    std::thread reader(receiver, std::ref(shared), port, cpu);
    const Scheduling scheduling = schedule(20, cpu);
    std::fprintf(stderr, "LiDAR guard 100 Hz, stale=%d ms, native motor gate; policy=%d priority=%d runmask=%u schedule_error=%d affinity_error=%d\n",
                 STALE_MS, scheduling.policy, scheduling.priority, scheduling.runmask, scheduling.error, scheduling.affinity_error);
    const auto start = Clock::now();
    auto next = start, next_status = start;
    uint64_t ticks = 0, late_ticks = 0, missed_ticks = 0, contention = 0;
    double maximum_lateness = 0;
    Sample sample;
    char message[2048], nearest[48], age_text[48], inferred_nearest[48], lidar_nearest[48];
    while (!stopping && (!seconds || Clock::now() - start < std::chrono::seconds(seconds))) {
        std::this_thread::sleep_until(next);
        const auto now = Clock::now();
        const double lateness = std::max(0.0, millis(now, next));
        maximum_lateness = std::max(maximum_lateness, lateness);
        if (lateness > 1.0) ++late_ticks;
        if (lateness >= 10.0) { missed_ticks += uint64_t(lateness / 10.0); next = now; }
        next += milliseconds(10);
        ++ticks;
        const bool locked = pthread_mutex_trylock(&shared.mutex) == 0;
        if (locked) { sample = shared.sample; pthread_mutex_unlock(&shared.mutex); }
        else ++contention;
        const bool connected = shared.connected.load(std::memory_order_acquire);
        const double age = sample.frames ? std::max(0.0, millis(now, sample.received)) : -1;
        const bool stale = !connected || age < 0 || age >= STALE_MS || sample.backlog_ms > 100;
        const char* verdict = "UNKNOWN";
        if (stale) verdict = "STALE";
        else if (locked) verdict = sample.verdict == Verdict::Blocked ? "BLOCKED" :
            sample.verdict == Verdict::Clear ? "CLEAR" : sample.verdict == Verdict::Caution ? "CAUTION" : "UNKNOWN";
        const char* lidar_verdict = verdict;
        const bool lidar_permit = !stale && locked && (sample.verdict == Verdict::Clear || sample.verdict == Verdict::Caution);
        const bool received_inferred_stop = fusion.poll();
        const uint64_t fusion_now = host_us();
        const bool inferred_live = fusion.live(fusion_now);
        const double inferred_distance = inferred_live ? fusion.nearest : -1;
        const bool inferred_stop = received_inferred_stop ||
            (inferred_distance >= 0 && inferred_distance <= double(.6f));
        const bool permit = lidar_permit && !inferred_stop;
        const char* stop_source = !lidar_permit ? "lidar" : inferred_stop ? "inferred" : "none";
        if (lidar_permit && inferred_stop) verdict = "BLOCKED";
        double combined_nearest = !stale && locked ? sample.nearest : -1;
        if (inferred_distance >= 0 && (combined_nearest < 0 || inferred_distance < combined_nearest))
            combined_nearest = inferred_distance;
        motor_gate(permit);
        if (now >= next_status || stopping) {
            next_status = now + milliseconds(50);
            if (combined_nearest >= 0) std::snprintf(nearest, sizeof nearest, "%.4f", combined_nearest);
            else std::strcpy(nearest, "null");
            if (inferred_distance >= 0) std::snprintf(inferred_nearest, sizeof inferred_nearest, "%.4f", inferred_distance);
            else std::strcpy(inferred_nearest, "null");
            if (!stale && locked && sample.nearest >= 0) std::snprintf(lidar_nearest, sizeof lidar_nearest, "%.4f", sample.nearest);
            else std::strcpy(lidar_nearest, "null");
            if (age >= 0) std::snprintf(age_text, sizeof age_text, "%.3f", age);
            else std::strcpy(age_text, "null");
            const int length = std::snprintf(message, sizeof message,
                "{\"type\":\"guard\",\"version\":1,\"verdict\":\"%s\",\"permit\":%s,\"advisory_only\":%s,\"motor_gate_enabled\":%s,"
                "\"lidar_verdict\":\"%s\",\"stop_source\":\"%s\",\"lidar_nearest_m\":%s,"
                "\"inferred_live\":%s,\"inferred_nearest_m\":%s,\"inferred_packets\":%llu,\"inferred_invalid\":%llu,"
                "\"fusion_enabled\":%s,\"sample_age_ms\":%s,\"nearest_m\":%s,\"nearest_obstacle_m\":%s,\"stop_distance_m\":0.6,\"warning_distance_m\":1.2,\"source_us\":%llu,\"seq\":%u,\"frames\":%llu,"
                "\"map_supported\":%s,\"points\":%u,\"floor_hits\":[%u,%u,%u],\"backlog_ms\":%.3f,\"connected\":%s,"
                "\"ticks\":%llu,\"tick_hz\":100,\"late_ticks\":%llu,\"missed_ticks\":%llu,\"max_lateness_ms\":%.3f,"
                "\"snapshot_contention\":%llu,\"invalid_packets\":%llu,\"uptime_ms\":%.3f,"
                "\"policy\":%d,\"priority\":%d,\"requested_priority\":20,\"requested_cpu\":%d,\"runmask\":%u,"
                "\"schedule_error\":%d,\"affinity_error\":%d}\n",
                verdict, permit ? "true" : "false", motor_port ? "false" : "true", motor_port ? "true" : "false",
                lidar_verdict, stop_source, lidar_nearest, inferred_live ? "true" : "false", inferred_nearest,
                (unsigned long long)fusion.accepted, (unsigned long long)fusion.invalid, fusion_port ? "true" : "false",
                age_text, nearest, nearest,
                (unsigned long long)sample.source_us, sample.seq, (unsigned long long)sample.frames,
                sample.map_supported ? "true" : "false", sample.points, sample.floor_hits[0], sample.floor_hits[1], sample.floor_hits[2], sample.backlog_ms,
                connected ? "true" : "false", (unsigned long long)ticks, (unsigned long long)late_ticks,
                (unsigned long long)missed_ticks, maximum_lateness, (unsigned long long)contention,
                (unsigned long long)shared.invalid.load(), millis(now, start), scheduling.policy, scheduling.priority,
                cpu, scheduling.runmask, scheduling.error, scheduling.affinity_error);
            if (length > 0 && size_t(length) < sizeof message)
                sendto(output, message, size_t(length), 0, reinterpret_cast<sockaddr*>(&target), sizeof target);
        }
    }
    motor_gate(false);
    stopping = 1;
    shared.connected.store(false, std::memory_order_release);
    // Explicit final stale datagram; absence of later heartbeats must also gate
    // the consumer. The separate native motor gate also expires without heartbeats.
    const char final[] = "{\"type\":\"guard\",\"version\":1,\"verdict\":\"STALE\",\"permit\":false,\"advisory_only\":false,\"connected\":false,\"reason\":\"guard exiting\"}\n";
    sendto(output, final, sizeof final - 1, 0, reinterpret_cast<sockaddr*>(&target), sizeof target);
    reader.join();
    close(output);
    return 0;
}
