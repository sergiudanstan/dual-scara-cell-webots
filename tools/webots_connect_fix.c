/* macOS 27: a refused non-blocking connect() makes the next connect() return EISCONN.
   Qt treats that as a live peer, so Webots R2025a refuses every port.
   Call the kernel directly, and if there is still no peer, report ECONNREFUSED. */
#define _DARWIN_C_SOURCE
#include <errno.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <unistd.h>

static int my_connect(int fd, const struct sockaddr *addr, socklen_t len) {
    int rc = syscall(SYS_connect, fd, addr, len);
    if (rc < 0 && errno == EISCONN) {
        struct sockaddr_storage peer;
        socklen_t plen = sizeof(peer);
        if (getpeername(fd, (struct sockaddr *)&peer, &plen) != 0) {
            errno = ECONNREFUSED;
            return -1;
        }
    }
    return rc;
}

typedef struct { const void *replacement; const void *replacee; } interpose_t;
__attribute__((used)) static const interpose_t interposers[]
    __attribute__((section("__DATA,__interpose"))) = {
        { (const void *)my_connect, (const void *)connect },
};
