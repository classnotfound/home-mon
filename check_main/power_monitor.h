#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the power monitor (configures GPIO13 and attaches the interrupt).
void power_monitor_init(void);

// RTOS task that monitors power supply and sends an alert SMS on power loss.
void power_monitor_task(void *pvParameters);

// Returns true if power is OK (GPIO13 is HIGH).
bool power_status(void);

extern volatile bool g_powerAlarm;

#ifdef __cplusplus
}
#endif

#endif // POWER_MONITOR_H
