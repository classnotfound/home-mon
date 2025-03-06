#include "battery_monitor.h"
#include <Arduino.h>
#include "esp_log.h"

#define TAG "BATTERY_MONITOR"

// ADC pin and resistor divider values.
// For example, using a resistor divider with R4 = 100 kΩ and R5 = 22 kΩ.
#define BATTERY_ADC_PIN 39    // Example ADC channel (GPIO39)
#define R4 100000.0            // Resistor between battery + and ADC input (100 kΩ)
#define R5 22000.0            // Resistor between ADC input and ground (22 kΩ)

// Global variable to hold the averaged battery voltage.
volatile float g_batteryVoltage = 0.0;

static float readBatteryVoltage() {
    const int numSamples = 10;  // Number of ADC samples to average.
    long sum = 0;
    for (int i = 0; i < numSamples; i++) {
        sum += analogRead(BATTERY_ADC_PIN);
        delay(10);  // Short delay between samples for stability.
    }
    float avgRaw = sum / (float)numSamples;
    // Convert the raw ADC value to voltage at the ADC pin (assuming 12-bit ADC and 3.3V reference).
    float adcVoltage = avgRaw * (3.3 / 4095.0);

    // Reverse the voltage divider calculation to get the battery voltage.
    float batteryVoltage = adcVoltage * ((R4 + R5) / R5);
    ESP_LOGI(TAG, "Voltage (avg): %.2f", batteryVoltage);
    return batteryVoltage;
}

void battery_monitor_task(void *pvParameters) {
    ESP_LOGI(TAG, "Starting battery monitor task.");
    for (;;) {
        float voltage = readBatteryVoltage();
        g_batteryVoltage = voltage;
        ESP_LOGI(TAG, "Measured battery voltage: %.2f V", voltage);
        vTaskDelay(pdMS_TO_TICKS(5000));  // Measure every 5 seconds.
    }
}

void battery_monitor_init(void) {
    // Initialize the ADC pin if needed.
    pinMode(BATTERY_ADC_PIN, INPUT);
}
