#include <stdint.h>
#include "battery_monitor.h"
#include <Arduino.h>
#include "esp_log.h"

#define TAG "BATTERY_MONITOR"

// ADC pin and resistor divider values.
// For example, using a resistor divider with R4 = 100 kΩ and R5 = 22 kΩ.
#define BATTERY_ADC_PIN 39    // Example ADC channel (GPIO39)
#define R4 10000.0            // Resistor between battery + and ADC input (10 kΩ)
#define R5 2700.0            // Resistor between ADC input and ground (2.7 kΩ)

#include "esp_adc_cal.h"

static esp_adc_cal_characteristics_t adc_chars;


// Global variable to hold the averaged battery voltage.
volatile float g_batteryVoltage = 0.0;

// Number of samples for the rolling average.
#define NUM_BATTERY_SAMPLES 10
static int batterySamples[NUM_BATTERY_SAMPLES] = {0};
static int currentSample = 0;

static float convertRawToVoltage(int rawValue) {
    // Convert raw ADC value (0-4095) to voltage at the ADC pin.
    float adcVoltage = rawValue * (3.3 / 4095.0);
    // Reverse the voltage divider: battery voltage = ADC voltage * ((R1+R2)/R2)
    float batteryVoltage = adcVoltage * ((R4 + R5) / R5);

    ESP_LOGD(TAG, "Raw ADC: %d, ADC Voltage: %.2f V, Calculated Battery Voltage: %.2f V", rawValue, adcVoltage, batteryVoltage);
    return batteryVoltage;
}

float battery_monitor_getVoltage() {
    long sum = 0;
    for (int i = 0; i < NUM_BATTERY_SAMPLES; i++) {
        sum += batterySamples[i];
    }
    float avgRaw = sum / (float)NUM_BATTERY_SAMPLES;
    return convertRawToVoltage(avgRaw);
}

void battery_monitor_task(void *pvParameters) {
    ESP_LOGI(TAG, "Starting battery monitor task with rolling average.");
    
    // Initialize the batterySamples array with an initial reading.
    int initial = analogRead(BATTERY_ADC_PIN);
    ESP_LOGI(TAG, "Initial ADC reading on pin %d: %d", BATTERY_ADC_PIN, initial);
    for (int i = 0; i < NUM_BATTERY_SAMPLES; i++) {
        batterySamples[i] = initial;
    }
    currentSample = 0;
    
    for (;;) {
        //garbage!
        analogRead(BATTERY_ADC_PIN);
        delay(200);
        analogRead(BATTERY_ADC_PIN);
        delay(200);
        // Take a single ADC measurement.
        int raw = analogRead(BATTERY_ADC_PIN);
        ESP_LOGD(TAG, "New ADC reading: %d", raw);
        batterySamples[currentSample] = raw;
        currentSample = (currentSample + 1) % NUM_BATTERY_SAMPLES;
        
        // Update the global battery voltage using the rolling average.
        g_batteryVoltage = battery_monitor_getVoltage();
        ESP_LOGI(TAG, "Averaged Battery Voltage: %.2f V", g_batteryVoltage);
        
        // Delay before taking the next measurement (e.g., 5 seconds).
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

void battery_monitor_init(void) {
    pinMode(BATTERY_ADC_PIN, INPUT);
    // Initialize the ADC pin if needed.
    analogSetAttenuation(ADC_11db);
    //esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    analogSetWidth(ADC_WIDTH_BIT_12);
    
    ESP_LOGI(TAG, "Battery monitor initialized on ADC pin %d", BATTERY_ADC_PIN);
}
