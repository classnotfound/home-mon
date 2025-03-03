#include "power_monitor.h"
#include <Arduino.h>
#include "esp_log.h"
#include "sim900.h"      // For sim900_send_sms and g_smsRecipient
#include "status_led.h"  // For g_powerAlarm

#define TAG "POWER_MONITOR"
#define POWER_PIN 13
#define POWER_DEBOUNCE_MS 500

volatile bool powerInterruptFlag = false;
static bool powerAlarmSent = false;

static void IRAM_ATTR powerISR(void)
{
    if (digitalRead(POWER_PIN) == LOW) {
      powerInterruptFlag = true;
    } else {
      powerInterruptFlag = false;
    }
}

void power_monitor_init(void)
{
    ESP_LOGI(TAG, "Initializing power monitor on GPIO %d", POWER_PIN);
    pinMode(POWER_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(POWER_PIN), powerISR, CHANGE);
}

bool power_status(void)
{
    return digitalRead(POWER_PIN) == HIGH;
}

void power_monitor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting power monitor task.");
    for (;;) {
        if (powerInterruptFlag) {
            ESP_LOGD(TAG, "Power interrupt flag set.");
            powerInterruptFlag = false;
            vTaskDelay(pdMS_TO_TICKS(POWER_DEBOUNCE_MS));
            if (digitalRead(POWER_PIN) == LOW) {
                // Power loss detected
                if (!powerAlarmSent) {
                    ESP_LOGI(TAG, "Power loss confirmed.");
                    if (g_alertingEnabled) {
                        sim900_send_alert("ALERT: Power loss detected!");
                    } else {
                        ESP_LOGI(TAG, "Alerting disabled; power loss SMS not sent.");
                    }
                    powerAlarmSent = true;
                    g_powerAlarm = true;
                }
            }
        }

        // Check if power is restored and send SMS if necessary
        if (digitalRead(POWER_PIN) == HIGH && powerAlarmSent) {
            ESP_LOGI(TAG, "Power restored.");
            if (g_alertingEnabled) {
                sim900_send_alert("ALERT: Power has been restored!");
            } else {
                ESP_LOGI(TAG, "Alerting disabled; power restored SMS not sent.");
            }
            powerAlarmSent = false;
            g_powerAlarm = false;
        }

        vTaskDelay(pdMS_TO_TICKS(180000));
    }
}
