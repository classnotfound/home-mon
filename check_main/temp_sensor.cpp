#include "temp_sensor.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Arduino.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sim900.h"      // for sim900_send_sms
#include "status_led.h"  // for g_tempAlarm

#define TAG "TEMP_SENSOR"
#define DS18B20_PIN 32

// Temperature thresholds and hysteresis.
#define TEMP_LOW_THRESHOLD   5.0f
#define TEMP_HIGH_THRESHOLD  70.0f
#define TEMP_HYSTERESIS       1.0f
#define TEMP_SAMPLE_COUNT    5

// Create a OneWire instance for DS18B20 on the defined pin.
OneWire oneWire(DS18B20_PIN);
// Create a DallasTemperature object to communicate with DS18B20.
DallasTemperature sensors(&oneWire);

// The temperature alert queue is created in main.cpp.
extern QueueHandle_t xTempAlertQueue;

typedef enum {
    ALERT_NONE = 0,
    ALERT_LOW,
    ALERT_HIGH
} tempAlertType_t;

// Global variable used to indicate an active temperature alarm.
extern volatile bool g_tempAlarm;

// Global variable to hold the averaged temperature.
volatile float g_temperatureAvg = DEVICE_DISCONNECTED_C;

static SemaphoreHandle_t sensorMutex = NULL;

void temp_sensor_init(void)
{
  ESP_LOGI(TAG, "Initializing DallasTemperature sensor.");
  sensors.begin();
  // Optionally, set sensor resolution (default is usually 12-bit).
  sensors.setResolution(12);
  // Create the sensor mutex if it doesn't exist.
  if (sensorMutex == NULL) {
    sensorMutex = xSemaphoreCreateMutex();
    if (sensorMutex == NULL) {
      ESP_LOGE(TAG, "Failed to create sensor mutex!");
    }
  }
}

float temp_sensor_read(void)
{
  float temp = DEVICE_DISCONNECTED_C;
  ESP_LOGD(TAG, "Requesting temperature conversion.");
  if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
    sensors.requestTemperatures();  // Send command to all sensors on the bus.
    temp = sensors.getTempCByIndex(0);
    if (temp == DEVICE_DISCONNECTED_C) {
      ESP_LOGE(TAG, "Error: DS18B20 sensor disconnected!");
    } else {
      ESP_LOGI(TAG, "Temperature read: %.2f C", temp);
    }
    xSemaphoreGive(sensorMutex);
  } else {
    ESP_LOGE(TAG, "Failed to take sensor mutex!");
  }
  return temp;
}

void temp_sensor_task_measure(void *pvParameters)
{
  ESP_LOGI(TAG, "Starting temperature measurement task.");
  temp_sensor_init(); // Initialize the sensor.
  float sampleSum = 0;
  int sampleCounter = 0;
  static tempAlertType_t lastAlert = ALERT_NONE;
  
  for (;;) {
    float temp = temp_sensor_read();
    if (temp != DEVICE_DISCONNECTED_C) {  // Valid reading.
      sampleSum += temp;
  
      sampleCounter++;
      if (sampleCounter >= TEMP_SAMPLE_COUNT) {
        float avgTemp = sampleSum / sampleCounter;
        ESP_LOGI(TAG, "Averaged Temperature: %.2f C", avgTemp);
        g_temperatureAvg = avgTemp;  // Update the global average.
        sampleSum = 0;
        sampleCounter = 0;
        
        if (avgTemp < TEMP_LOW_THRESHOLD && lastAlert != ALERT_LOW) {
          ESP_LOGI(TAG, "Temperature below threshold (%.2f < %.2f)", avgTemp, TEMP_LOW_THRESHOLD);
          tempAlertType_t alert = ALERT_LOW;
          xQueueSend(xTempAlertQueue, &alert, 0);
          lastAlert = ALERT_LOW;
          g_tempAlarm = true;
        } else if (avgTemp > TEMP_HIGH_THRESHOLD && lastAlert != ALERT_HIGH) {
          ESP_LOGI(TAG, "Temperature above threshold (%.2f > %.2f)", avgTemp, TEMP_HIGH_THRESHOLD);
          tempAlertType_t alert = ALERT_HIGH;
          xQueueSend(xTempAlertQueue, &alert, 0);
          lastAlert = ALERT_HIGH;
          g_tempAlarm = true;
        } else if (avgTemp >= (TEMP_LOW_THRESHOLD + TEMP_HYSTERESIS) &&
                    avgTemp <= (TEMP_HIGH_THRESHOLD - TEMP_HYSTERESIS)) {
          if (lastAlert != ALERT_NONE) {
            ESP_LOGI(TAG, "Temperature returned to normal range (%.2f)", avgTemp);
          }
          lastAlert = ALERT_NONE;
          g_tempAlarm = false;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(300000));
  }
}

void temp_sensor_task_alert(void *pvParameters)
{
  ESP_LOGI(TAG, "Starting temperature alert task.");
  tempAlertType_t alert;
  char msg[128];
  for (;;) {
    if (xQueueReceive(xTempAlertQueue, &alert, portMAX_DELAY) == pdTRUE) {
      if (alert == ALERT_LOW)
        snprintf(msg, sizeof(msg), "ALERT: Temperature LOW: %.2f C", g_temperatureAvg);
      else if (alert == ALERT_HIGH)
        snprintf(msg, sizeof(msg), "ALERT: Temperature HIGH: %.2f C", g_temperatureAvg);
      ESP_LOGI(TAG, "Temperature alert: %s", msg);
      if (g_alertingEnabled) {
        // Use centralized alerting.
        sim900_send_alert(msg);
      } else {
        ESP_LOGI(TAG, "Alerting disabled; SMS not sent.");
      }
    }
  }
}
