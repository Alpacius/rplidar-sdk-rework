#pragma once

#include    <include/compile-utils.h>

#ifndef RPLIDAR_SDK_RE_ARCH
#error "Architecture undefined - missing macro definition: RPLIDAR_SDK_RE_ARCH"
#endif

#ifndef RPLIDAR_SDK_RE_IOTYPE
#error "I/O type undefined - missing macro definition: RPLIDAR_SDK_RE_IOTYPE"
#endif

#if P_ARCH == ARCH_LINUX

#if P_IOTYPE == IOTYPE_SERIAL
#include <include/arch/linux/serial.h>
#else
#error "I/O type unsupported"
#endif

#else
#error "Architecture unsupported"
#endif

#if P_IOTYPE == IOTYPE_SERIAL

#define io_descriptor serial_descriptor

#define init_descriptor init_descriptor_serial
#define ruin_descriptor ruin_descriptor_serial
#define new_descriptor new_descriptor_serial
#define delete_descriptor delete_descriptor_serial

#define send_raw send_serial
#define recv_raw recv_serial

#define stop_motor stop_motor_serial
#define start_motor start_motor_serial

#else
#error "I/O type unsupported"
#endif
