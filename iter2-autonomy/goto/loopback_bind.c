/* Scope the prebuilt QNX goto command listener to the local dashboard.
 * Loaded only in the supervised goto process to retain the verified executable. */
#include <arpa/inet.h>
#include <dlfcn.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>

int bind(int fd, const struct sockaddr *addr, socklen_t len)
{
    typedef int (*bind_fn)(int, const struct sockaddr *, socklen_t);
    bind_fn original = (bind_fn)dlsym(RTLD_NEXT, "bind");
    if (!original) { errno = ENOSYS; return -1; }
    if (addr && addr->sa_family == AF_INET && len == sizeof(struct sockaddr_in)) {
        struct sockaddr_in local;
        memcpy(&local, addr, sizeof local);
        if (local.sin_addr.s_addr == htonl(INADDR_ANY)) {
            local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            return original(fd, (struct sockaddr *)&local, sizeof local);
        }
    }
    return original(fd, addr, len);
}
