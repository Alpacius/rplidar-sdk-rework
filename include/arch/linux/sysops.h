#pragma once

#include    <stdlib.h>
#include    <string.h>
#include    <stdarg.h>
#include    <unistd.h>

#define delay_yield(ms_) usleep((ms_) * 1000)
