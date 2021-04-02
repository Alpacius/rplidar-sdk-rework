#include    <include/rplidar.h>
#include    <include/protocol.h>

#ifndef RPLIDAR_SDK_ASYNC
#include <src/rplidar_cmd_sync.c>
#else
#error "Async mode not supported"
#endif
