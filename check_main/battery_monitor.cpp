#include "battery_monitor.h"
#include <Arduino.h>
#include "esp_log.h"
#include "sim900.h" // For sim900_send_alert()

#define TAG "BATTERY_MONITOR"

// ADC pin for battery measurement (using GPIO39).
#define BATTERY_ADC_PIN 39

// Resistor divider values (example: using 10kΩ and 2.7kΩ)
#define R4 10000.0   // Resistor between battery+ and ADC input
#define R5 2700.0    // Resistor between ADC input and ground

// Global variable holding the averaged battery voltage.
volatile float g_batteryVoltage = 0.0;

// Global flag for battery alert state.
volatile bool g_batteryAlarm = false;

// Number of samples for the rolling average.
#define NUM_BATTERY_SAMPLES 10
static int batterySamples[NUM_BATTERY_SAMPLES] = {0};
static int currentSample = 0;

// Battery voltage thresholds (in Volts)
#define BATTERY_LOW_THRESHOLD 8.0
#define BATTERY_HYSTERESIS 1.0  // So, restored when voltage > 9.0 V

// Helper function: Convert a raw ADC reading to battery voltage.
static float convertRawToVoltage(int rawValue) {
  // Convert raw ADC (0-4095) to voltage at ADC pin (assuming a 3.3 V reference).
  float adcVoltage = rawValue * (3.3 / 4095.0);
  // Reverse the resistor divider:
  float batteryVoltage = adcVoltage * ((R4 + R5) / R5);
  ESP_LOGD(TAG, "Raw ADC: %d, ADC Voltage: %.2f V, Calculated Battery Voltage: %.2f V", 
            rawValue, adcVoltage, batteryVoltage);
  return batteryVoltage;
}

// Calculates the average battery voltage from the rolling buffer.
float battery_monitor_getVoltage(void) {
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
    // Take a single ADC measurement.
    int raw = analogRead(BATTERY_ADC_PIN);
    ESP_LOGD(TAG, "New ADC reading: %d", raw);
    batterySamples[currentSample] = raw;
    currentSample = (currentSample + 1) % NUM_BATTERY_SAMPLES;
    
    // Compute the averaged voltage.
    g_batteryVoltage = battery_monitor_getVoltage();
    ESP_LOGI(TAG, "Averaged Battery Voltage: %.2f V", g_batteryVoltage);
    
    // Check battery voltage threshold with hysteresis.
    if (g_batteryVoltage < BATTERY_LOW_THRESHOLD && !g_batteryAlarm) {
      ESP_LOGI(TAG, "Battery voltage below threshold: %.2f V", g_batteryVoltage);
      char msg[64];
      snprintf(msg, sizeof(msg), "ALERT: Battery voltage low: %.2f V", g_batteryVoltage);
      sim900_send_alert(msg);
      g_batteryAlarm = true;
    } else if (g_batteryVoltage > (BATTERY_LOW_THRESHOLD + BATTERY_HYSTERESIS) && g_batteryAlarm) {
      ESP_LOGI(TAG, "Battery voltage restored: %.2f V", g_batteryVoltage);
      char msg[64];
      snprintf(msg, sizeof(msg), "ALERT: Battery voltage normal: %.2f V", g_batteryVoltage);
      sim900_send_alert(msg);
      g_batteryAlarm = false;
    }
    
    // Delay before next measurement (e.g., 30 seconds).
    vTaskDelay(pdMS_TO_TICKS(30000));
  }
}

void battery_monitor_init(void) {
  pinMode(BATTERY_ADC_PIN, INPUT);
  ESP_LOGI(TAG, "Battery monitor initialized on ADC pin %d", BATTERY_ADC_PIN);
}
