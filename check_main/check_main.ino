#include <Arduino.h>
#include "sim900.h"
#include "temp_sensor.h"
#include "power_monitor.h"
#include "battery_monitor.h"
#include "status_led.h"
#include "time_monitor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include <WiFi.h>
#include "esp_bt.h"

// Create the temperature alert queue.
QueueHandle_t xTempAlertQueue;

// Define global alarm flags so they can be shared among modules.
volatile bool g_tempAlarm = false;
volatile bool g_powerAlarm = false;
volatile bool contactNotFound = false;

void led_update_task(void *pvParameters)
{
    ESP_LOGI("LED_UPDATE", "Starting LED update task.");
    for (;;) {
        if (contactNotFound)
          g_alarm_led_off_delay = 600;  
        else
          g_alarm_led_off_delay = ((g_tempAlarm || g_powerAlarm) ? 1000 : 5000);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, 16, 17); // Adjust baud rate as needed for SIM900.
    ESP_LOGI("MAIN", "System starting up.");
    esp_bt_controller_disable();
    WiFi.mode(WIFI_OFF);      // Disable WiFi
    
    // Wake up SIM900.
    //ESP_LOGD("MAIN", "Wake up sim900.");
    //sim900_wakeup();
    validateStateOn();
    ESP_LOGD("MAIN", "Checking sim900 status.");
    sim900_is_ready();
    ESP_LOGD("MAIN", "The sim900 should be ready.");

    time_monitor_init();
    // Retrieve contacts from SIM (indices 99 to 104).
    int nbContacts = sim900_get_contacts();
    ESP_LOGD("MAIN", "Got %d contact(s) from sim900, start monitoring task.", nbContacts);
    xTaskCreate(sim900_task_monitor, "SIM900MonitorTask", 4096, NULL, 5, NULL);
    // Create the temperature alert queue.
    xTempAlertQueue = xQueueCreate(10, sizeof(int));
    if (xTempAlertQueue == NULL) {
        ESP_LOGE("MAIN", "Failed to create temperature alert queue.");
    }
    
    // Create FreeRTOS tasks.
    xTaskCreate(sim900_task_check_sms, "SMSCheckTask", 8192, NULL, 2, NULL);
    xTaskCreate(temp_sensor_task_measure, "TempMeasureTask", 4096, NULL, 5, NULL);
    xTaskCreate(temp_sensor_task_alert, "TempAlertTask", 4096, NULL, 5, NULL);
    power_monitor_init();
    xTaskCreate(power_monitor_task, "PowerMonitorTask", 4096, NULL, 5, NULL);
    battery_monitor_init();
    xTaskCreate(battery_monitor_task, "BatteryMonitorTask", 4096, NULL, 5, NULL);
    xTaskCreate(board_status_led_task, "BoardLEDTask", 2048, NULL, 1, NULL);
    xTaskCreate(alarm_status_led_task, "AlarmLEDTask", 2048, NULL, 1, NULL);
    xTaskCreate(led_update_task, "LEDUpdateTask", 2048, NULL, 1, NULL);
    
    // After initialization, set board LED to slow blink.
    g_board_led_off_delay = 3000;
    ESP_LOGI("MAIN", "System initialization complete.");
}

void loop() {
    // All work is handled by FreeRTOS tasks.
}
