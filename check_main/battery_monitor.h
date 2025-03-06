#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

// Global variable holding the latest measured battery voltage.
extern volatile float g_batteryVoltage;

// Initializes the battery monitor.
void battery_monitor_init(void);

// RTOS task that periodically measures and averages the battery voltage.
void battery_monitor_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // BATTERY_MONITOR_H
