/**
 * @file syscalls.c
 * @brief Minimal newlib syscall stubs for bare-metal STM32.
 *
 * Provides the POSIX-like functions that newlib expects but are not
 * available on a bare-metal target. _write is implemented separately
 * in debug_output.c (redirected to USB CDC).
 */

#include <sys/stat.h>
#include <errno.h>

int _close(int fd)
{
    (void)fd;
    errno = EBADF;
    return -1;
}

int _fstat(int fd, struct stat *st)
{
    (void)fd;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int fd)
{
    (void)fd;
    return 1;
}

int _lseek(int fd, int offset, int whence)
{
    (void)fd;
    (void)offset;
    (void)whence;
    errno = ESPIPE;
    return -1;
}

int _read(int fd, char *ptr, int len)
{
    (void)fd;
    (void)ptr;
    (void)len;
    return 0;
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}
