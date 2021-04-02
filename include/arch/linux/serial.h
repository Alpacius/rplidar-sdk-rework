#pragma once

#include    <stdlib.h>
#include    <string.h>
#include    <stdarg.h>
#include    <unistd.h>
#include    <fcntl.h>
#include    <errno.h>

#include    <sys/ioctl.h>
#ifdef __GNUC__
#include    <asm/ioctls.h>
#include    <asm/termbits.h>
extern int tcflush(int, int);
#else
#error "__GNUC__ undefined"
#endif

#include    <include/protocol.h>

struct serial_descriptor {
    int fd;
    speed_t baud_rate;
};

int init_descriptor_serial(struct serial_descriptor *d, ...);
int ruin_descriptor_serial(struct serial_descriptor *d);
size_t send_serial(struct serial_descriptor *d, uint8_t *buf, size_t size);
size_t recv_serial(struct serial_descriptor *d, uint8_t *buf, size_t size);

int start_motor_serial(struct serial_descriptor *d);
int stop_motor_serial(struct serial_descriptor *d);

#define new_descriptor_serial(...) \
    ({ \
        __auto_type d_ = malloc(sizeof(struct serial_descriptor)); \
        d_ ? (init_descriptor_serial(d_, __VA_ARGS__) == 0 ? d_ : (free(d_), NULL)) : NULL; \
    })
#define delete_descriptor_serial(d_) free((d_))
