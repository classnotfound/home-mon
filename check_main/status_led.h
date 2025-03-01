#ifndef STATUS_LED_H
#define STATUS_LED_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

// Task for controlling the board status LED (GPIO26).
void board_status_led_task(void *pvParameters);

// Task for controlling the alarm status LED (GPIO27).
void alarm_status_led_task(void *pvParameters);

// Global off-delay variables (in ms) used by the LED tasks.
extern volatile uint32_t g_board_led_off_delay;  // for board LED
extern volatile uint32_t g_alarm_led_off_delay;   // for alarm LED

#ifdef __cplusplus
}
#endif

#endif // STATUS_LED_H
