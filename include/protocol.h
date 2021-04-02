#pragma once

#include    <stdint.h>
#include    <stddef.h>

struct rplidar_request {
    uint8_t start_flag;
    uint8_t command;
    uint8_t data[];
} __attribute__((packed));

#define START_FLAG_REQ              0xA5

struct rplidar_response_descriptor {
    uint8_t start_flag_1;
    uint8_t start_flag_2;
    union {
        struct {
            uint32_t send_mode : 2;
            uint32_t length : 30;
        } __attribute__((packed)) pack;
        uint32_t raw;
    } payload_info;
    uint8_t data_type;
} __attribute__((packed));

#define START_FLAG_RESP_INVALID     0
#define START_FLAG_RESP_1           0xA5
#define START_FLAG_RESP_2           0x5A

#define SEND_MODE_SREQ_SRESP        0
#define SEND_MODE_SREQ_MRESP        1
#define SEND_MODE_RESERVED_1        2
#define SEND_MODE_RESERVED_2        3

// Basic commands supported by all firmware versions.
#define CMD_STOP                    0x25
#define CMD_RESET                   0x40
#define CMD_SCAN                    0x20
#define CMD_FORCE_SCAN              0x21
#define CMD_GET_INFO                0x50
#define CMD_GET_HEALTH              0x52

// Extensions since firmware version 1.17.
#define CMD_EXPRESS_SCAN            0x82
#define CMD_GET_SAMPLE_RATE         0x59

// Extensions since firmware version 1.24.
#define CMD_GET_LIDAR_CONF          0x84

// Declarations for flyweight requests.
extern struct rplidar_request *cmd_stop;
extern struct rplidar_request *cmd_reset;
extern struct rplidar_request *cmd_scan;
extern struct rplidar_request *cmd_force_scan;
extern struct rplidar_request *cmd_get_info;
extern struct rplidar_request *cmd_get_health;

#define req_payload_len(req_p_) \
    ({ \
        uint8_t len_ = 0; \
        const struct rplidar_request *req_p_v_ = (req_p_); \
        uint8_t cmd_v_ = (req_p_)->command; \
        switch (cmd_v_) { \
            case CMD_EXPRESS_SCAN: \
                len_ = 5; \
                break; \
            case CMD_GET_LIDAR_CONF: \
                len_ = 4; \
                { \
                    uint32_t conf_type_ = ((uint32_t *) req_p_v_->data)[0]; \
                    switch (conf_type_) { \
                        case 0x70: \
                        case 0x7C: break; \
                        case 0x71: \
                        case 0x74: \
                        case 0x75: \
                        case 0x7F: len_ += 2; break; \
                    } \
                } \
                break; \
            default: \
                ; \
        } \
        len_; \
    })

#define req_needs_checksum(req_p_) \
    ({ \
        const struct rplidar_request *req_p_v_ = (req_p_); \
        ((req_p_v_->command == CMD_EXPRESS_SCAN) || (req_p_v_->command == CMD_GET_LIDAR_CONF)); \
    })

#define req_single_response_len(req_p_) \
    ({ \
        ssize_t len_ = 0; \
        const struct rplidar_request *req_p_v_ = (req_p_); \
        uint8_t cmd_v_ = (req_p_)->command; \
        switch (cmd_v_) { \
            case CMD_GET_INFO: \
                len_ = 20; \
                break; \
            case CMD_GET_HEALTH: \
                len_ = 3; \
                break; \
            case CMD_GET_SAMPLE_RATE: \
                len_ = 4; \
                break; \
            case CMD_GET_LIDAR_CONF: \
                { \
                    uint32_t conf_type_ = ((uint32_t *) req_p_v_->data)[0]; \
                    switch (conf_type_) { \
                        case 0x70: len_ = 2; break; \
                        case 0x71: len_ = 4; break; \
                        case 0x74: len_ = 4; break; \
                        case 0x75: len_ = 1; break; \
                        case 0x7C: len_ = 2; break; \
                        case 0x7F: len_ = -1; break; \
                    } \
                } \
            default: \
                ; \
        } \
        len_; \
    })

// TODO CMD_EXPRESS_SCAN
#define req_streaming_response_len(req_p_) \
    ({ \
        ssize_t len_ = -1; \
        const struct rplidar_request *req_p_v_ = (req_p_); \
        uint8_t cmd_v_ = (req_p_)->command; \
        switch (cmd_v_) { \
            case CMD_SCAN: \
            case CMD_FORCE_SCAN: len_ = 5; break; \
        } \
        len_; \
    })

static inline
uint8_t request_checksum(const struct rplidar_request *r) {
    uint8_t payload_len = req_payload_len(r);
    if (payload_len == 0)
        return 0;

    uint8_t xorsum = 0;
    xorsum = 0;
    xorsum ^= r->start_flag;
    xorsum ^= r->command;
    xorsum ^= payload_len;
    for (uint8_t i = 0; i < payload_len; i++)
        xorsum ^= r->data[i];

    return xorsum;
}

struct scan_response_packet {
    union {
        struct {
            uint8_t new_scan: 1;
            uint8_t new_scan_: 1;
            uint8_t quality: 6;
        } __attribute__((packed)) pack;
        uint8_t raw;
    } scan_meta;
    union {
        struct {
            uint8_t always_one: 1;
            uint8_t q6_0_6: 7;
        } __attribute__((packed)) pack;
        uint8_t raw;
    } angle_q6_0_6;
    uint8_t angle_q6_7_14;
    uint8_t distance_q2_0_7;
    uint8_t distance_q2_8_15;
} __attribute__((packed));

