#pragma once

#define unlikely(e_) __builtin_expect(!!(e_), 0)
#define likely(e_) __builtin_expect(!!(e_), 1)

#define prep_cat(arg, ...) prep_cat_impl(arg, __VA_ARGS__)
#define prep_cat_impl(arg, ...) arg ## __VA_ARGS__

#define ARCH_LINUX      1

#define IOTYPE_SERIAL   1
#define IOTYPE_UDP      2

#define P_ARCH          prep_cat(ARCH_, RPLIDAR_SDK_RE_ARCH)
#define P_IOTYPE        prep_cat(IOTYPE_, RPLIDAR_SDK_RE_IOTYPE)
