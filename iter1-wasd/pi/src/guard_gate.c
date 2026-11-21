#include "guard_gate.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define GATE_TTL_US 100000ULL
static int fd = -1, enabled, permit, obstacle;
static uint64_t stamp;
static const char *reason = "disabled";
static uint64_t now_us(void) {
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000ULL + (uint64_t)t.tv_nsec / 1000;
}
static uint64_t be64(const unsigned char *p) {
    uint64_t n=0; for (int i=0;i<8;i++) n=(n<<8)|p[i]; return n;
}
int guard_gate_init(int port) {
    enabled = port != 0; permit = 0; stamp = 0;
    if (!enabled) return 0;
    reason = "guard_missing";
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags|O_NONBLOCK) < 0) { guard_gate_close(); return -1; }
    struct sockaddr_in a; memset(&a,0,sizeof a);
    a.sin_family=AF_INET; a.sin_addr.s_addr=htonl(INADDR_LOOPBACK); a.sin_port=htons((unsigned short)port);
    if (bind(fd,(struct sockaddr *)&a,sizeof a)<0) { guard_gate_close(); return -1; }
    return 0;
}
int guard_gate_poll(void) {
    if (!enabled) return 1;
    unsigned char packet[32];
    int denied_this_poll = 0, invalid_this_poll = 0, other_denial = 0;
    /* Small fixed packets; bounded work even if a faulty sender floods us.
       Timestamp uses this host's CLOCK_MONOTONIC, so queued/replayed CLEAR
       packets cannot extend a lease. Localhost is a trust boundary, not auth. */
    for (int i=0;i<16;i++) {
        ssize_t n=recv(fd,packet,sizeof packet,0);
        if (n<0) break;
        uint64_t now=now_us();
        int classified = n == 13 && !memcmp(packet,"GOB1",4) && packet[12] == 0;
        if (n!=13 || (!classified && memcmp(packet,"GAT1",4)) || packet[12]>1) {
            permit=0; denied_this_poll=1; invalid_this_poll=1; reason="guard_invalid"; continue;
        }
        uint64_t sent=be64(packet+4);
        if (sent<=stamp || sent>now || now-sent>=GATE_TTL_US) continue;
        stamp=sent; permit=packet[12]; obstacle=classified;
        if (!permit && !classified) other_denial=1;
        if (!permit) denied_this_poll=1;
        reason=permit ? "clear" : "guard_blocked";
    }
    int stale = !stamp || now_us()-stamp>=GATE_TTL_US;
    if (stale) {
        permit=0; reason=stamp ? "guard_stale" : "guard_missing";
    }
    /* Only a well-formed, fresh BLOCKED may read as "guard_blocked": tars_motor
       lets an in-place TURN through on that reason, so an invalid packet or a
       stale/missing guard must never be relabelled as blocked. */
    if (invalid_this_poll || stale || other_denial) obstacle=0;
    if (invalid_this_poll) { reason="guard_invalid"; return 0; }
    if (denied_this_poll) { if (!stale) reason="guard_blocked"; return 0; }
    if (permit) reason="clear";
    return permit;
}
int guard_gate_obstacle(void) { return obstacle; }
long guard_gate_age_ms(void) { return stamp ? (long)((now_us()-stamp)/1000) : -1; }
const char *guard_gate_reason(void) { return reason; }
void guard_gate_close(void) { if(fd>=0) close(fd); fd=-1; }
