#include    <include/protocol.h>
#include    <include/rplidar.h>

// Flyweight definitions for requests with no payload.

static struct rplidar_request cmd_stop_ = { .start_flag = START_FLAG_REQ, .command = CMD_STOP };
static struct rplidar_request cmd_reset_ = { .start_flag = START_FLAG_REQ, .command = CMD_RESET };
static struct rplidar_request cmd_scan_ = { .start_flag = START_FLAG_REQ, .command = CMD_SCAN };
static struct rplidar_request cmd_force_scan_ = { .start_flag = START_FLAG_REQ, .command = CMD_FORCE_SCAN };
static struct rplidar_request cmd_get_info_ = { .start_flag = START_FLAG_REQ, .command = CMD_GET_INFO };
static struct rplidar_request cmd_get_health_ = { .start_flag = START_FLAG_REQ, .command = CMD_GET_HEALTH };

struct rplidar_request *cmd_stop = &cmd_stop_;
struct rplidar_request *cmd_reset = &cmd_reset_;
struct rplidar_request *cmd_scan = &cmd_scan_;
struct rplidar_request *cmd_force_scan = &cmd_force_scan_;
struct rplidar_request *cmd_get_info = &cmd_get_info_;
struct rplidar_request *cmd_get_health = &cmd_get_health_;

