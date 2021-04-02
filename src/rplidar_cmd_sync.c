#include    <stdint.h>
#include    <stddef.h>

#include    <include/compile-utils.h>
#include    <include/error_no.h>
#include    <include/rplidar.h>
#include    <include/protocol.h>
#include    <include/raw-io.h>
#include    <include/sysops.h>

#define discard_request(r_) ((r_)->cmd.in = NULL)
#define discard_header_out(r_) ((r_)->cmd.header_out.start_flag_1 = START_FLAG_RESP_INVALID)
#define setup_drain_action(r_, s_, n_) \
    do { \
        struct rplidar *r_v_ = (r_); \
        r_v_->cmd.drain.state = (s_); \
        r_v_->cmd.drain.count = (n_); \
    } while (0)
#define clear_drain_action(r_) \
    do { \
        struct rplidar *r_v_ = (r_); \
        r_v_->cmd.drain.state = DRAIN_STATE_NONE; \
        r_v_->cmd.drain.count = 0; \
    } while (0)
#define should_drain(r_) ((r_)->cmd.drain.state != DRAIN_STATE_NONE)

static uint8_t drain_buffer[256];

// Try to drain what's left, given the request sent.
// Failure means the device state is out of control from the perspective of the SDK, thus ERR_DRAIN shall be set.
// The invariant here is that oob_stop() is performed before any drain operation.
static
ssize_t drain(struct rplidar *r) {
    switch (r->cmd.drain.state) {
        case DRAIN_STATE_NONE:
            clear_drain_action(r);
            return 0;
        case DRAIN_STATE_HEADER:
            {
                size_t drain_size = recv_raw(&r->io, drain_buffer, r->cmd.drain.count);
                if (drain_size == r->cmd.drain.count) {
                    setup_drain_action(r, DRAIN_STATE_PAYLOAD, 0);
                } else {
                    // Keep the messy state, set up error_no and fail.
                    rplidar_set_error(r, RPLIDAR_ERR_DRAIN);
                    return -1;
                }
            }
            // Fallthrough
        case DRAIN_STATE_PAYLOAD:
            switch (r->cmd.in->command) {
                case CMD_SCAN:
                    // TODO implementation
                    rplidar_cough_no_impl(r);
                    break;
                case CMD_EXPRESS_SCAN:
                    // TODO implementation
                    rplidar_cough_no_impl(r);
                    break;
                case CMD_GET_LIDAR_CONF:
                    // TODO implementation
                    rplidar_cough_no_impl(r);
                    break;
                default:
                    ;
            }
            clear_drain_action(r);
            return 0;
        default:
            ;
    }
    return -1;
}

// Try to stop any further operations on the lidar side.
// Failure means the device state is out of control from the perspective of the SDK, thus ERR_DRAIN shall be set.
static
int oob_stop(struct rplidar *r) {
    if (send_raw(&r->io, (uint8_t *) cmd_stop, sizeof(struct rplidar_request)) != sizeof(struct rplidar_request))
        return rplidar_set_error(r, RPLIDAR_ERR_DRAIN), -1;
    return 0;
}

// "do_" functions are candidates for their corresponding API implementations when asynchronization support is disabled.
// When async features are enabled, these synchronized operations shall be reused to support async mode.

// API substitution shall be header-neutral. That is, declarations in either a header file or this source file shall cause no conflict.

int do_emit_command(struct rplidar *r, struct rplidar_request *req) {
    if (r->cmd.in == NULL) {
        r->cmd.in = req;
        size_t req_size = sizeof(struct rplidar_request) + req_payload_len(req) + req_needs_checksum(req);
        if (send_raw(&r->io, (uint8_t *) req, req_size) == 0)
            return 0;
    }
    return -1;
}
#ifndef emit_command
#define emit_command do_emit_command
#endif

int do_retrieve_response_header(struct rplidar *r) {
    struct rplidar_request *req = r->cmd.in;
    if (req != NULL) {
        // Drain what's left.
        if (unlikely(should_drain(r) && drain(r) == -1))
            return discard_header_out(r), discard_request(r), -1;

        size_t n_hdr_recv = recv_raw(&r->io, (uint8_t *) &(r->cmd.header_out), sizeof(struct rplidar_response_descriptor));
        if (n_hdr_recv == sizeof(struct rplidar_response_descriptor)) {
            return 0;
        } else {
            // Header incomplete. All its following contents shall be discarded, and a quick stop shall be performed
            // to stop further flooding of payloads.
            // However, stop actions may fail. It is up to the superiors to decide what to do next when a fatal error occurs.
            setup_drain_action(r, DRAIN_STATE_HEADER, sizeof(struct rplidar_response_descriptor) - n_hdr_recv);
            oob_stop(r);
            return discard_header_out(r), discard_request(r), -1;
        }
    }
    return -1;
}
#ifndef retrieve_response_header
#define retrieve_response_header do_retrieve_response_header
#endif

int do_prepare_single_payload(struct rplidar *r) {
    struct rplidar_request *req = r->cmd.in;
    ssize_t payload_size = req_single_response_len(req);

    switch (payload_size) {
        case -1:
            discard_request(r);
            discard_header_out(r);
            // TODO implementation
            rplidar_cough_no_impl(r);
            return -1;
        case 0:
            return discard_header_out(r), discard_request(r), 0;
        default:
            {
                size_t n_pld_recv = recv_raw(&r->io, r->cmd.resp_data_buffer.data, (size_t) payload_size);
                if (n_pld_recv == (size_t) payload_size) {
                    return 0;
                } else {
                    setup_drain_action(r, DRAIN_STATE_PAYLOAD, payload_size - ((size_t) n_pld_recv));
                    oob_stop(r);
                    return discard_header_out(r), discard_request(r), -1;
                }
            }
            return 0;
    }
    return -1;
}
#ifndef prepare_single_payload
#define prepare_single_payload do_prepare_single_payload
#endif

int do_prepare_streaming_payloads(struct rplidar *r, sync_buf_idx_t n_expected, sync_buf_idx_t *n_actual) {
    struct rplidar_request *req = r->cmd.in;
    ssize_t packet_len = req_streaming_response_len(req);

    if (unlikely(packet_len < 1))
        return rplidar_cough_no_impl(r), -1;

    sync_buf_idx_t n_avail = (RPLIDAR_SDK_SYNC_BUFFER_SIZE - r->cmd.resp_data_buffer.size) / packet_len;
    if (unlikely(n_avail == 0))
        return rplidar_set_error(r, RPLIDAR_ERR_AGAIN), -1;

    sync_buf_idx_t n_packets = n_expected <= n_avail ? n_expected : n_avail;
    size_t n_bytes_expected = n_packets * packet_len;
    size_t n_bytes_recv = recv_raw(&r->io, r->cmd.resp_data_buffer.data + r->cmd.resp_data_buffer.size, n_bytes_expected);
    if (unlikely(n_bytes_recv % packet_len)) {
        oob_stop(r);
        return discard_header_out(r), discard_request(r), -1;
    }
    r->cmd.resp_data_buffer.size += n_bytes_recv;
    *n_actual = n_bytes_recv / packet_len;
    return 0;
}
#ifndef prepare_streaming_payloads
#define prepare_streaming_payloads do_prepare_streaming_payloads
#endif

int do_finish_command(struct rplidar *r) {
    return discard_request(r), discard_header_out(r), 0;
}
#ifndef finish_command
#define finish_command do_finish_command
#endif
