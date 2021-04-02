#pragma once

#define RPLIDAR_ERR_NONE        0
#define RPLIDAR_ERR_NO_IMPL     1       // No implementation for the feature.
#define RPLIDAR_ERR_DRAIN       2       // Fail to perform any drain action. The device is totally out of control due to communication issues and needs resetting.
#define RPLIDAR_ERR_AGAIN       3       // Unable to perform I/O operation due to little buffering space.
