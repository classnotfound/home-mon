#include "status_led.h"
#include <Arduino.h>
#include "esp_log.h"

#define BOARD_LED_PIN 26
#define ALARM_LED_PIN 27

#define TAG "STATUS_LED"

// Global off-delay variables (defaults: fast blink initially).
volatile uint32_t g_board_led_off_delay = 800;
volatile uint32_t g_alarm_led_off_delay = 2800;

void board_status_led_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting board status LED task on GPIO %d", BOARD_LED_PIN);
    pinMode(BOARD_LED_PIN, OUTPUT);
    for (;;) {
        //ESP_LOGD(TAG, "Board LED ON (200ms)");
        digitalWrite(BOARD_LED_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(200));
        digitalWrite(BOARD_LED_PIN, LOW);
        //ESP_LOGD(TAG, "Board LED OFF for %u ms", g_board_led_off_delay);
        vTaskDelay(pdMS_TO_TICKS(g_board_led_off_delay));
    }
}

void alarm_status_led_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting alarm status LED task on GPIO %d", ALARM_LED_PIN);
    pinMode(ALARM_LED_PIN, OUTPUT);
    for (;;) {
        //ESP_LOGD(TAG, "Alarm LED ON (200ms)");
        digitalWrite(ALARM_LED_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(200));
        digitalWrite(ALARM_LED_PIN, LOW);
        //ESP_LOGD(TAG, "Alarm LED OFF for %u ms", g_alarm_led_off_delay);
        vTaskDelay(pdMS_TO_TICKS(g_alarm_led_off_delay));
    }
}
