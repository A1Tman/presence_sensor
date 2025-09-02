#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** Store detected sensor (radar) firmware version string. */
void sensor_info_set_fw(const char *fw);

/** Get last known sensor firmware version string; returns empty string if unknown. */
const char *sensor_info_get_fw(void);

#ifdef __cplusplus
}
#endif

