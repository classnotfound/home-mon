#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the DS18B20 sensor using DallasTemperature.
void temp_sensor_init(void);

// Reads the temperature (in °C) from the DS18B20 sensor.
float temp_sensor_read(void);

// RTOS task for periodically measuring temperature and averaging samples.
void temp_sensor_task_measure(void *pvParameters);

// RTOS task for sending SMS alerts when temperature thresholds are exceeded.
void temp_sensor_task_alert(void *pvParameters);

// Global average temperature computed in the measurement task.
extern volatile float g_temperatureAvg;

#ifdef __cplusplus
}
#endif

#endif // TEMP_SENSOR_H
