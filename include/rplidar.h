#pragma once

#include    <stddef.h>
#include    <stdint.h>

#include    <include/raw-io.h>
#include    <include/protocol.h>
#include    <include/error_no.h>

#ifndef RPLIDAR_SDK_SYNC_BUFFER_SIZE
#define RPLIDAR_SDK_SYNC_BUFFER_SIZE 8192
#endif

#if RPLIDAR_SDK_SYNC_BUFFER_SIZE <= 256
#define sync_buf_idx_t uint8_t
#elif RPLIDAR_SDK_SYNC_BUFFER_SIZE <= 65536
#define sync_buf_idx_t uint16_t
#elif RPLIDAR_SDK_SYNC_BUFFER_SIZE <= 4294967296
#define sync_buf_idx_t uint32_t
#else
#error "Sync buffer too large"
#endif

struct rplidar_cmd_buffer {
#ifndef RPLIDAR_SDK_ASYNC
    struct rplidar_request *in;
    struct rplidar_response_descriptor header_out;
    struct {
        uint8_t data[RPLIDAR_SDK_SYNC_BUFFER_SIZE];
        sync_buf_idx_t size;
    } resp_data_buffer;
    struct {
        uint8_t count;
        uint8_t state;
    } drain;
#else
#error "Async mode not supported"
#endif
};

#ifndef RPLIDAR_SDK_ASYNC
#define DRAIN_STATE_NONE 0
#define DRAIN_STATE_HEADER 1
#define DRAIN_STATE_PAYLOAD 2
#endif

static inline
void init_rplidar_cmd_buffer(struct rplidar_cmd_buffer *cmd) {
#ifndef RPLIDAR_SDK_ASYNC
    (cmd->in = NULL), 
    (cmd->header_out.start_flag_1 = START_FLAG_RESP_INVALID), (cmd->resp_data_buffer.size = 0), 
    (cmd->drain.count = 0), (cmd->drain.state = DRAIN_STATE_NONE);
#else
#error "Async mode not supported"
#endif
}

static inline
void ruin_rplidar_cmd_buffer(struct rplidar_cmd_buffer *cmd) {
    // TODO implementation
}

struct rplidar {
    uint8_t version;
    uint8_t error_no;
    struct io_descriptor io;
    struct rplidar_cmd_buffer cmd;
#ifdef RPLIDAR_SDK_ASYNC
    int commu[2];
#endif
};

#define VERSION_A1 0
#define VERSION_A2 1

static inline
int rplidar_start_motor(struct rplidar *r) {
    return start_motor(&r->io);
}

static inline
int rplidar_stop_motor(struct rplidar *r) {
    return stop_motor(&r->io);
}

#ifndef RPLIDAR_SDK_ASYNC
#define rplidar_init(r_, v_, ...) \
    ({ \
        struct rplidar *r_v_ = (r_); \
        r_v_->version = (v_); \
        init_descriptor(&r_v_->io, __VA_ARGS__); \
        init_rplidar_cmd_buffer(&r_v_->cmd); \
    })
#else
#error "Async mode not supported"
#endif

#define rplidar_set_error(r_, e_) ((r_)->error_no = (e_))
#define rplidar_clear_error(r_) ((r_)->error_no = RPLIDAR_ERR_NONE)
#define rplidar_cough_no_impl(r_) ((r_)->error_no = RPLIDAR_ERR_NO_IMPL)
