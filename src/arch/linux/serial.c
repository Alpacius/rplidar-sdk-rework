#include    <include/arch/linux/serial.h>
#include    <include/arch/linux/sysops.h>

static
int open_serial(const char *path, speed_t baud_rate) {
    int fd = open(path, O_RDWR|O_NOCTTY|O_NDELAY);
    if (fd == -1)
        return -1;

    struct termios2 tios;
    ioctl(fd, TCGETS2, &tios);
    bzero(&tios, sizeof(struct termios2));

    tios.c_cflag = BOTHER;
    tios.c_cflag |= (CLOCAL|CREAD|CS8);
    tios.c_cflag &= ~CSTOPB;
    tios.c_cflag &= ~CRTSCTS;
    tios.c_cflag &= ~PARENB;
#ifdef CNEW_RTSCTS
    tios.c_cflag &= ~CNEW_RTSCTS;
#endif

    tios.c_iflag &= ~(IXON|IXOFF|IXANY);

    tios.c_cc[VMIN] = 0;
    tios.c_cc[VTIME] = 0;

    tios.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);

    tios.c_oflag &= ~OPOST;

    tios.c_ispeed = baud_rate;
    tios.c_ospeed = baud_rate;

    if (ioctl(fd, TCSETS2, &tios) == -1) {
        close(fd);
        return -1;
    }

    if (tcflush(fd, TCIFLUSH) == -1) {
        close(fd);
        return -1;
    }
    if (fcntl(fd, F_SETFL, FNDELAY) == -1) {
        close(fd);
        return -1;
    }

    return fd;
}

int init_descriptor_serial(struct serial_descriptor *d, ...) {
    const char *path;
    speed_t baud_rate;
    va_list ap;
    va_start(ap, d);
    {
        path = va_arg(ap, const char *);
        baud_rate = va_arg(ap, speed_t);
    }
    va_end(ap);

    d->baud_rate = baud_rate;
    if ((d->fd = open_serial(path, baud_rate)) == -1)
        return -1;
    unsigned dtrbit = TIOCM_DTR;
    if (ioctl(d->fd, TIOCMBIC, &dtrbit) == -1) 
        return -1;
    if (tcflush(d->fd, TCIFLUSH) == -1)
        return -1;
    if (ioctl(d->fd, TIOCMBIS, &dtrbit) == -1)
        return -1;
    sleep(5);
    if (tcflush(d->fd, TCIFLUSH) == -1)
        return -1;
    return 0;
}

int ruin_descriptor_serial(struct serial_descriptor *d) {
    return close(d->fd);
}

size_t send_serial(struct serial_descriptor *d, uint8_t *buf, size_t size) {
    size_t tx_bytes = 0;
    do {
        ssize_t actual_tx_bytes = write(d->fd, buf + tx_bytes, size - tx_bytes);
        if (actual_tx_bytes < 0)
            break;
        tx_bytes += actual_tx_bytes;
    } while (tx_bytes < size);
    return tx_bytes;
}

size_t recv_serial(struct serial_descriptor *d, uint8_t *buf, size_t size) {
    size_t rx_bytes = 0;
    do {
        ssize_t actual_rx_bytes = read(d->fd, buf + rx_bytes, size - rx_bytes);
        if (actual_rx_bytes < 0)
            break;
        rx_bytes += actual_rx_bytes;
    } while (rx_bytes < size);
    return rx_bytes;
}

int start_motor_serial(struct serial_descriptor *d) {
    unsigned dtrbit = TIOCM_DTR;
    if (ioctl(d->fd, TIOCMBIS, &dtrbit) == -1)
        return -1;
    return delay_yield(500);
}

int stop_motor_serial(struct serial_descriptor *d) {
    unsigned dtrbit = TIOCM_DTR;
    if (ioctl(d->fd, TIOCMBIC, &dtrbit) == -1)
        return -1;
    return delay_yield(500);
}
