#include "sensor_info.h"
#include <string.h>

static char s_fw[64];

void sensor_info_set_fw(const char *fw)
{
    if (!fw) return;
    size_t n = strlen(fw);
    if (n >= sizeof(s_fw)) n = sizeof(s_fw) - 1;
    memcpy(s_fw, fw, n);
    s_fw[n] = '\0';
}

const char *sensor_info_get_fw(void)
{
    return s_fw[0] ? s_fw : "";
}

