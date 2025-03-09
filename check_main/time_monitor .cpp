#include "time_monitor.h"
#include "sim900.h"      // To call sim900_get_time()
#include "esp_log.h"
#include <string.h>

#define TAG "TIME_MONITOR"

// Global variable to hold the startup time.
char g_startTime[32] = "Unknown";

bool time_monitor_init(void) {
    ESP_LOGI(TAG, "Initializing time monitor. Querying SIM900 network time...");
    if (sim900_get_time(g_startTime, sizeof(g_startTime))) {
         ESP_LOGI(TAG, "Device start time: %s", g_startTime);
         return true;
    }
    ESP_LOGE(TAG, "Failed to retrieve network time from SIM900.");
    return false;
}
