#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

// Global variable holding the averaged battery voltage (in Volts).
extern volatile float g_batteryVoltage;

// Global flag that indicates whether a low-voltage alarm is active.
extern volatile bool g_batteryAlarm;

// Initializes the battery monitor hardware.
void battery_monitor_init(void);

// RTOS task that periodically reads the battery voltage using a rolling average and sends alerts.
void battery_monitor_task(void *pvParameters);

// Returns the current averaged battery voltage.
float battery_monitor_getVoltage(void);

#ifdef __cplusplus
}
#endif

#endif // BATTERY_MONITOR_H
