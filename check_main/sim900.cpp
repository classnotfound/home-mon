#include "sim900.h"
#include <Arduino.h>
#include <string.h>
#include "esp_log.h"
#include "temp_sensor.h"   // for temp_sensor_read()
#include "power_monitor.h" // for power_status()
#include "status_led.h"


#define TAG "SIM900"
#define SIM900_WAKE_PIN 15

// Global variables (populated via SIM contact read)
char g_smsRecipient[32] = {0};
char g_smsPassword[64]    = {0};

//HardwareSerial sim900(2);

// Global flag for alerting (enabled by default)
volatile bool g_alertingEnabled = true;

#define SIM900_MAX 5
volatile int sim900NotResponsive=0;
volatile bool g_sim900Ready = false;


static void wakeSim900Hardware(void)
{
    ESP_LOGI("wake_Sim900Hardware", "Toggling wake pin (GPIO %d)", SIM900_WAKE_PIN);
    pinMode(SIM900_WAKE_PIN, OUTPUT);
    digitalWrite(SIM900_WAKE_PIN, HIGH);
    delay(1000);  // 1-second pulse
    digitalWrite(SIM900_WAKE_PIN, LOW);
    // Wait a few seconds for the module to do something?
    delay(5000);
    ESP_LOGI("wake_Sim900Hardware", "Wake pulse sent.");
}

static int sendATCommand(const char* command, char* response, int responseSize, const char* expectedSubstring, unsigned long timeout)
{
    ESP_LOGD(TAG, "Sending command: %s", command);
    while (Serial2.available()) { Serial2.read(); }
    Serial2.println(command);
    delay(500); 
    memset(response, 0, responseSize);
    int idx = 0;
    unsigned long start = millis();
    while (millis() - start < timeout) {
        while (Serial2.available()) {
            char c = char(Serial2.read());
            if (idx < responseSize - 1)
                response[idx++] = c;
        }
        if (strstr(response, expectedSubstring) != NULL) {
            ESP_LOGD(TAG, "Expected substring '%s' found in '%s'", expectedSubstring, response);
            sim900NotResponsive=0;
            return 1;
        }
        delay(10);
    }
    sim900NotResponsive++;
    ESP_LOGD(TAG, "Timeout (%lu ms) reached without expected response: '%s' (failed attempts: %d)", timeout, response, sim900NotResponsive);
    return 0;
}

void validateStateOn() {
  int elapsed = 0;
  const int interval = 1000;
  const int timeout_ms = 30000;
  while (elapsed < timeout_ms) {
      char buffer[128];
      if (sendATCommand("AT", buffer, sizeof(buffer), "OK", 5000)) {
          ESP_LOGI("sim900_wakeup", "SIM900 is ready after wakeup.");
          return;
      }
      /*if (sim900_is_ready()) {
          ESP_LOGI("sim900_wakeup", "SIM900 is ready after wakeup.");
          return;
      }*/
      delay(interval);
      elapsed += interval;
  }
  ESP_LOGE("sim900_wakeup", "SIM900 failed to become ready within timeout.");
}


void sim900_wakeup(void)
{
    ESP_LOGI("sim900_wakeup", "Waking up SIM900...");
    wakeSim900Hardware();
    
    validateStateOn();
}

bool sim900_is_ready(void)
{
    char buffer[128];
    if (!sendATCommand("AT", buffer, sizeof(buffer), "OK", 5000)) {
        ESP_LOGE("sim900_is__ready", "SIM900 not responding to AT command.");
        return false;
    }
    ESP_LOGI("sim900_is__ready", "SIM900 responded to AT command.");
    
    sendATCommand("AT+CFUN=1", buffer, sizeof(buffer), "OK", 2000);
    sendATCommand("AT+CSCLK=0", buffer, sizeof(buffer), "OK", 2000);
    sendATCommand("AT+CMGF=1", buffer, sizeof(buffer), "OK", 2000);  // Text Mode
    //sendATCommand("AT+CSCS=\"UCS2\""); //UCS2 encoding (sms!!)
    sendATCommand("AT+CNMI=2,0,0,0,0", buffer, sizeof(buffer), "OK", 2000);
    sendATCommand("AT+CPMS?", buffer, sizeof(buffer), "OK", 2000);

    // Check signal quality
    if (sendATCommand("AT+CSQ", buffer, sizeof(buffer), "OK", 2000)) {
        char *csqPtr = strstr(buffer, "+CSQ:");
        if (csqPtr) {
            int signalQuality = -1, ber = -1;
            if (sscanf(csqPtr, "+CSQ: %d,%d", &signalQuality, &ber) == 2)
                ESP_LOGI("sim900_is__ready", "Signal Quality: %d, BER: %d", signalQuality, ber);
            else
                ESP_LOGD("sim900_is__ready", "Unable to parse signal quality info.");
        }
    } else {
        ESP_LOGE("sim900_is__ready", "Failed to get signal quality from SIM900.");
    }
    
    // Check network registration
    if (sendATCommand("AT+CREG?", buffer, sizeof(buffer), "OK", 2000)) {
        char *cregPtr = strstr(buffer, "+CREG:");
        if (cregPtr) {
            int n = -1, stat = -1;
            if (sscanf(cregPtr, "+CREG: %d,%d", &n, &stat) == 2)
                ESP_LOGI("sim900_is__ready", "Network Registration: n=%d, stat=%d", n, stat);
            else
                ESP_LOGD("sim900_is__ready", "Unable to parse network registration info.");
        }
    } else {
        ESP_LOGE("sim900_is__ready", "Failed to get network registration info from SIM900.");
    }
    return true;
}

bool sim900_get_contact_info(int index, char* password, int pwdSize, char* phoneNumber, int phoneSize){
  if(g_sim900Ready) {
    ESP_LOGI(TAG, "Reading contact info at index %d", index);
    char buffer[256];
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CPBR=%d", index);
    if (!sendATCommand(cmd, buffer, sizeof(buffer), "OK", 3000)) {
        ESP_LOGE(TAG, "Failed to read contact at index %d.", index);
        return false;
    }
    // Expected response format: +CPBR: 99,"+1234567890",129,"password"
    char tmpPhone[64];
    char tmpName[64];
    int idx, type;
    ESP_LOGI(TAG, "Parsing string: '%s'", buffer);

    char *cpbrPtr = strstr(buffer, "+CPBR:");
    if(cpbrPtr != NULL) {
      if (sscanf(cpbrPtr, "+CPBR: %d,\"%[^\"]\",%d,\"%[^\"]\"", &idx, tmpPhone, &type, tmpName) == 4) {
          strncpy(phoneNumber, tmpPhone, phoneSize);
          phoneNumber[phoneSize - 1] = '\0';
          strncpy(password, tmpName, pwdSize);
          password[pwdSize - 1] = '\0';
          ESP_LOGI(TAG, "Contact info: Recipient=%s, Password=%s", phoneNumber, password);
          strncpy(g_smsRecipient, phoneNumber, sizeof(g_smsRecipient));
          g_smsRecipient[sizeof(g_smsRecipient)-1] = '\0';
          strncpy(g_smsPassword, password, sizeof(g_smsPassword));
          g_smsPassword[sizeof(g_smsPassword)-1] = '\0';
          if(strlen(password) > 0) {
            return true;
          }
          ESP_LOGE(TAG, "Password was empty!!");
          g_alarm_led_off_delay = 1600;
          return false;
      }
      ESP_LOGE(TAG, "Failed to parse contact info: %s", buffer);
      return false;
    } else {
      ESP_LOGE(TAG, "Could not find '+CPBR:' in the response: %s", buffer);
    }
  }
}

bool sim900_send_sms(const char* recipient, const char* message)
{
    g_sim900Ready=false;
    ESP_LOGI(TAG, "Sending SMS to %s", recipient);
    char buffer[128];
    if (!sendATCommand("AT+CMGF=1", buffer, sizeof(buffer), "OK", 2000)) {
        ESP_LOGE(TAG, "Failed to set text mode for SMS.");
        return false;
    }
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", recipient);
    //while (Serial2.available()) { Serial2.read(); }
    ESP_LOGD(TAG, "AT command: %s", cmd);
    Serial2.println(cmd);
    memset(buffer, 0, sizeof(buffer));
    int idx = 0;
    unsigned long start = millis();
    while (millis() - start < 5000) {
        while (Serial2.available()) {
            char c = Serial2.read();
            if (idx < (int)sizeof(buffer)-1){
                buffer[idx++] = c;
            }
        }
        if (strchr(buffer, '>') != NULL) {
            ESP_LOGD(TAG, "SMS prompt received.");
            break;
        }
        delay(10);
    }
    if (strchr(buffer, '>') == NULL) {
        ESP_LOGE(TAG, "No prompt received for SMS input.");
        return false;
    }
    Serial2.print(message);
    Serial2.write((char)26); // Send Ctrl+Z to terminate.
    memset(buffer, 0, sizeof(buffer));
    idx = 0;
    start = millis();
    while (millis() - start < 5000) {
        while (Serial2.available()) {
            char c = Serial2.read();
            if (idx < (int)sizeof(buffer)-1)
                buffer[idx++] = c;
        }
        if (strstr(buffer, "OK") != NULL || strstr(buffer, "+CMGS:") != NULL) {
            ESP_LOGI(TAG, "SMS sent successfully.");
            g_sim900Ready=true;
            return true;
        }
        delay(10);
    }
    ESP_LOGE(TAG, "Failed to send SMS.");
    g_sim900Ready=true;
    return false;
}

void sim900_send_status_sms(void) {
  if (g_sim900Ready) {
    ESP_LOGI(TAG, "Composing SIM900 status SMS.");
    char statusMessage[256];
    char buffer[128];
    int signalQuality = -1, ber = -1;
    int n = -1, stat = -1;
    
    if (sendATCommand("AT+CSQ", buffer, sizeof(buffer), "OK", 2000)) {
        char *csqPtr = strstr(buffer, "+CSQ:");
        if (csqPtr)
            sscanf(csqPtr, "+CSQ: %d,%d", &signalQuality, &ber);
    }
    if (sendATCommand("AT+CREG?", buffer, sizeof(buffer), "OK", 2000)) {
        char *cregPtr = strstr(buffer, "+CREG:");
        if (cregPtr)
            sscanf(cregPtr, "+CREG: %d,%d", &n, &stat);
    }
    snprintf(statusMessage, sizeof(statusMessage),
             "SIM900 Status:\nSignal: %d, BER: %d\nNetwork: n=%d, stat=%d",
             signalQuality, ber, n, stat);
    ESP_LOGI(TAG, "Status SMS message: %s", statusMessage);
    sim900_send_sms(g_smsRecipient, statusMessage);
  }
}

void sim900_send_full_status_sms(void)
{
    ESP_LOGI(TAG, "Composing full status SMS.");
    char statusMessage[512];
    char simStatus[256];
    char powerStatus[16];
    
    char buffer[128];
    int signalQuality = -1, ber = -1;
    int n = -1, stat = -1;
    if (sendATCommand("AT+CSQ", buffer, sizeof(buffer), "OK", 2000)) {
        char *csqPtr = strstr(buffer, "+CSQ:");
        if (csqPtr)
            sscanf(csqPtr, "+CSQ: %d,%d", &signalQuality, &ber);
    }
    if (sendATCommand("AT+CREG?", buffer, sizeof(buffer), "OK", 2000)) {
        char *cregPtr = strstr(buffer, "+CREG:");
        if (cregPtr)
            sscanf(cregPtr, "+CREG: %d,%d", &n, &stat);
    }
    snprintf(simStatus, sizeof(simStatus),
             "SIM900:\nSignal: %d, BER: %d\nNetwork: n=%d, stat=%d",
             signalQuality, ber, n, stat);
    
    float temp = temp_sensor_read();
    bool powerOk = power_status();
    snprintf(powerStatus, sizeof(powerStatus), "%s", powerOk ? "OK" : "Lost");
    
    snprintf(statusMessage, sizeof(statusMessage),
             "%s\nTemperature: %.2f C\nPower: %s",
             simStatus, temp, powerStatus);
    ESP_LOGI(TAG, "Full status SMS message: %s", statusMessage);
    sim900_send_sms(g_smsRecipient, statusMessage);
}

void sim900_task_check_sms(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting SMS check task.");
    char response[512];
    for (;;) {
      if(g_sim900Ready) {
        if (sendATCommand("AT+CMGL=\"REC UNREAD\"", response, sizeof(response), "OK", 2000)) {
            ESP_LOGD(TAG, "SMS list response: %s", response);
            // Check if the SMS contains the correct password.
            if (strlen(g_smsPassword) > 0 && strstr(response, g_smsPassword) != NULL) {
                ESP_LOGI(TAG, "SMS command identified (password match).");
                char *delimiter = strstr(response, " - ");
                if (delimiter) {
                    char commandBuffer[64];
                    strncpy(commandBuffer, delimiter + 3, sizeof(commandBuffer)-1);
                    commandBuffer[sizeof(commandBuffer)-1] = '\0';
                    char *newline = strchr(commandBuffer, '\r');
                    if (newline) *newline = '\0';
                    newline = strchr(commandBuffer, '\n');
                    if (newline) *newline = '\0';
                    ESP_LOGI(TAG, "Parsed command: %s", commandBuffer);
                    if (strcmp(commandBuffer, "enable alerting") == 0) {
                        g_alertingEnabled = true;
                        ESP_LOGI(TAG, "Alerting enabled via SMS command.");
                        sim900_send_sms(g_smsRecipient, "Alerting enabled.");
                    } else if (strcmp(commandBuffer, "disable alerting") == 0) {
                        g_alertingEnabled = false;
                        ESP_LOGI(TAG, "Alerting disabled via SMS command.");
                        sim900_send_sms(g_smsRecipient, "Alerting disabled.");
                    } else if (strcmp(commandBuffer, "status") == 0) {
                        ESP_LOGI(TAG, "Status command received.");
                        sim900_send_full_status_sms();
                    } else {
                        ESP_LOGW(TAG, "Unknown command received: %s", commandBuffer);
                    }
                }
            }
            sendATCommand("AT+CMGDA=\"DEL ALL\"", response, sizeof(response), "OK", 2000);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(51000));
    }
}

void sim900_task_monitor(void *pvParameters)
{
    ESP_LOGI("sim900__task_monitor", "Starting SIM900 monitor task.");
    for (;;) {
        ESP_LOGI("sim900__task_monitor", "Checking sim900 status %d", sim900NotResponsive>SIM900_MAX);
        if (sim900NotResponsive>SIM900_MAX) {
            g_sim900Ready = false;
            ESP_LOGE("sim900__task_monitor", "SIM900 unresponsive, performing reset...");
            sim900_wakeup();
            sim900NotResponsive = 0;
            /*delay(1500);
            if (sim900_is_ready()) {
                g_sim900Ready = true;
                ESP_LOGI(TAG, "SIM900 reset successfully.");
            } else {
                ESP_LOGE(TAG, "SIM900 still unresponsive after reset attempt.");
            }*/
            g_sim900Ready = true;
        } else {
          ESP_LOGD("sim900__task_monitor", "Nothing to do for the moment");
        }
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}
