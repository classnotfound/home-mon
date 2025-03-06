#include "sim900.h"
#include <Arduino.h>
#include <string.h>
#include "esp_log.h"
#include "temp_sensor.h"   // for temp_sensor_read()
#include "power_monitor.h" // for power_status()
#include "status_led.h"
#include "freertos/semphr.h"
#include "battery_monitor.h" // for g_batteryVoltage

static SemaphoreHandle_t sim900_mutex = NULL;

#define TAG "SIM900"
#define SIM900_WAKE_PIN 15

Contact g_contacts[5];

//HardwareSerial sim900(2);

// Global flag for alerting (enabled by default)
volatile bool g_alertingEnabled = true;

#define SIM900_MAX 2
volatile int sim900NotResponsive=0;



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
    // Take the mutex
    xSemaphoreTake(sim900_mutex, portMAX_DELAY);
    
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
            xSemaphoreGive(sim900_mutex);
            return 1;
        }
        delay(10);
    }
    sim900NotResponsive++;
    ESP_LOGD(TAG, "Timeout (%lu ms) reached without expected response: '%s' (failed attempts: %d)", timeout, response, sim900NotResponsive);
    xSemaphoreGive(sim900_mutex);
    return 0;
}

void validateStateOn(void) {
  sim900_mutex = xSemaphoreCreateMutex();
  if (sim900_mutex == NULL) {
      ESP_LOGE(TAG, "Failed to create SIM900 mutex!");
  } else {
    ESP_LOGD(TAG, "Created SIM900 mutex successfully!");
  }
  int elapsed = 0;
  const int interval = 1000;
  const int timeout_ms = 8000;
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

void sim900_task_monitor(void *pvParameters)
{
    ESP_LOGI("sim900__task_monitor", "Starting SIM900 monitor task.");
    for (;;) {
        ESP_LOGI("sim900__task_monitor", "Checking sim900 status %d", sim900NotResponsive>SIM900_MAX);
        if (sim900NotResponsive>SIM900_MAX) {
            xSemaphoreTake(sim900_mutex, portMAX_DELAY);
            ESP_LOGE("sim900__task_monitor", "SIM900 unresponsive, performing reset...");
            //Wait to let other commands finish
            vTaskDelay(pdMS_TO_TICKS(3000));
            sim900_wakeup();
            sim900NotResponsive = 0;
            xSemaphoreGive(sim900_mutex);
        } else {
          ESP_LOGD("sim900__task_monitor", "Nothing to do for the moment");
        }
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}


bool sim900_is_ready(void)
{
    char buffer[128];
    if (!sendATCommand("AT", buffer, sizeof(buffer), "OK", 10000)) {
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

int sim900_get_contacts(void)
{
    ESP_LOGI(TAG, "Retrieving contacts from SIM (indices 99 to 104)");
    char buffer[256];
    char cmd[32];
    int nbContacts = 0;
    for (int i = 0; i < 5; i++) {
        int index = 99 + i;
        snprintf(cmd, sizeof(cmd), "AT+CPBR=%d\r\n", index);
        if (sendATCommand(cmd, buffer, sizeof(buffer), "OK", 3000)) {
            // Extract the substring starting with "+CPBR:".
            char *cpbrPtr = strstr(buffer, "+CPBR:");
            if (cpbrPtr != NULL) {
                char tmpPhone[64], tmpName[64];
                int idx, type;
                if (sscanf(cpbrPtr, "+CPBR: %d,\"%[^\"]\",%d,\"%[^\"]\"", &idx, tmpPhone, &type, tmpName) == 4) {
                    strncpy(g_contacts[i].phone, tmpPhone, sizeof(g_contacts[i].phone));
                    g_contacts[i].phone[sizeof(g_contacts[i].phone)-1] = '\0';
                    strncpy(g_contacts[i].password, tmpName, sizeof(g_contacts[i].password));
                    g_contacts[i].password[sizeof(g_contacts[i].password)-1] = '\0';
                    g_contacts[i].defined = true;
                    nbContacts++;
                    ESP_LOGI(TAG, "Contact %d: Phone=%s, Password=%s", idx, g_contacts[i].phone, g_contacts[i].password);
                    continue;
                }
            }
        }
        g_contacts[i].defined = false;
        ESP_LOGI(TAG, "Contact at index %d not defined.", 99 + i);
    }
    return nbContacts;
}

// Centralized alert function: sends an alert SMS to all defined contacts.
void sim900_send_alert(const char* msg)
{
    ESP_LOGI(TAG, "Sending alert SMS to all contacts: %s", msg);
    for (int i = 0; i < 5; i++) {
        if (g_contacts[i].defined) {
            sim900_send_sms(g_contacts[i].phone, msg);
        }
    }
}

bool sim900_send_sms(const char* recipient, const char* message)
{
    ESP_LOGI(TAG, "Sending SMS to %s", recipient);
    char buffer[128];
    if (!sendATCommand("AT+CMGF=1", buffer, sizeof(buffer), "OK", 2000)) {
        ESP_LOGE(TAG, "Failed to set text mode for SMS.");
        return false;
    }
    char cmd[64];

    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", recipient);
    
    ESP_LOGD(TAG, "AT command: %s", cmd);
    
    // Take the mutex
    xSemaphoreTake(sim900_mutex, portMAX_DELAY);
    
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
        xSemaphoreGive(sim900_mutex);
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
            xSemaphoreGive(sim900_mutex);
            return true;
        }
        delay(10);
    }
    ESP_LOGE(TAG, "Failed to send SMS.");
    xSemaphoreGive(sim900_mutex);
    return false;
}

int countDefinedContacts(void) {
  int nbContacts = 0;
  for (int i = 0; i < 5; i++) {
    if (g_contacts[i].defined) {
      nbContacts++;
    }
  }
  return nbContacts;
}

void sim900_send_full_status_sms_to(const char* recipient)
{
    ESP_LOGI(TAG, "Composing full status SMS for %s.", recipient);
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
    
    bool powerOk = power_status();
    snprintf(powerStatus, sizeof(powerStatus), "%s", powerOk ? "OK" : "Lost");
    int nbContacts = countDefinedContacts();
    snprintf(statusMessage, sizeof(statusMessage),
             "%s\nTemperature: %.2f C\nPower: %s\nBattery voltage: %.2f\nNb contacts: %d",
             simStatus, g_temperatureAvg, powerStatus, g_batteryVoltage, nbContacts);
    ESP_LOGI(TAG, "Full status SMS message: %s", statusMessage);
    sim900_send_sms(recipient, statusMessage);
}

// Parses the SMS response to identify the contact and extract the command.
// Returns the index of the matching contact (0-4) if found, or -1 otherwise.
// The extracted command (after " - ") is copied into cmdBuffer.
static int identify_contact_from_sms(const char* smsResponse, char* cmdBuffer, int cmdBufferSize) {
  // Make a copy of the response because strtok modifies the string.
  char responseCopy[512];
  strncpy(responseCopy, smsResponse, sizeof(responseCopy)-1);
  responseCopy[sizeof(responseCopy)-1] = '\0';
  
  char *line = strtok(responseCopy, "\n");
  while (line != NULL) {
    if (strstr(line, " - ") != NULL) {
      ESP_LOGD(TAG, "Candidate SMS line: %s", line);
      for (int i = 0; i < 5; i++) {
        if (g_contacts[i].defined && strlen(g_contacts[i].password) > 0) {
          if (strncmp(line, g_contacts[i].password, strlen(g_contacts[i].password)) == 0) {
            // Found a matching contact.
            char *delimiter = strstr(line, " - ");
            if (delimiter) {
              strncpy(cmdBuffer, delimiter + 3, cmdBufferSize-1);
              cmdBuffer[cmdBufferSize-1] = '\0';
              ESP_LOGD(TAG, "Command found: %s", cmdBuffer);
              // Remove trailing newline characters.
              char *nl = strchr(cmdBuffer, '\r');
              if (nl) *nl = '\0';
              ESP_LOGD(TAG, "After 1: %s", cmdBuffer);
              nl = strchr(cmdBuffer, '\n');
              if (nl) *nl = '\0';
              
              ESP_LOGD(TAG, "Command before trim: %s", cmdBuffer);
              // Remove trailing spaces
              size_t len = strlen(cmdBuffer);
              ESP_LOGD(TAG, "Command length: %d", len);
              while (len > 0 && isspace((unsigned char)cmdBuffer[len-1])) {
                cmdBuffer[len-1] = '\0';
                len--;
                ESP_LOGD(TAG, "Command during trim: %s", cmdBuffer);
              }
              ESP_LOGD(TAG, "Command after trim: %s", cmdBuffer);
              return i;
            }
          }
        }
      }
    }
    line = strtok(NULL, "\n");
  }
  return -1;
}
static bool sim900_delete_contact(int index) {
  char cmd[128];
  char response[128];
  // Delete the existing contact (set it empty...).
  snprintf(cmd, sizeof(cmd), "AT+CPBW=%d", index);
  if (!sendATCommand(cmd, response, sizeof(response), "OK", 3000)) {
    ESP_LOGW(TAG, "Failed to delete contact at index %d", index);
    return false;
  } else {
    ESP_LOGI(TAG, "Deleted contact at index %d", index);
    return true;
  }
}

static bool sim900_update_contact(int index, const char* name, const char* number) {
  char cmd[128];
  char response[128];
  //not needed!
  //sim900_delete_contact(index);
  // Write the new contact.
  // Here we use type 129 (commonly used for international format).
  snprintf(cmd, sizeof(cmd), "AT+CPBW=%d,\"%s\",129,\"%s\"", index, number, name);
  if (sendATCommand(cmd, response, sizeof(response), "OK", 3000)) {
    ESP_LOGI(TAG, "Updated contact at index %d: Name=%s, Number=%s", index, name, number);
    return true;
  } else {
    ESP_LOGE(TAG, "Failed to update contact at index %d", index);
    return false;
  }
}


// Helper function to process the update command.
// The expected command format (after the initial password and " - " delimiter):
// "update contact <INDEX> <NAME> <NUM>"
// For example: "update 100 Popeye +33612345678"
static void process_update_command(const char* command, int senderContactIndex) {
    int newIndex;
    char newName[64];
    char newPhone[32];
    
    // The command string starts with "update contact", so skip the first 15 characters.
    int n = sscanf(command + 15, "%d %63s %31s", &newIndex, newName, newPhone);
    
    if (n == 3) {
      if (newIndex >= 99 && newIndex <= 104) {
        if (sim900_update_contact(newIndex, newName, newPhone)) {
          int arrayIndex = newIndex - 99;
          // Update the global contact array with the new values.
          strncpy(g_contacts[arrayIndex].password, newName, sizeof(g_contacts[arrayIndex].password));
          g_contacts[arrayIndex].password[sizeof(g_contacts[arrayIndex].password)-1] = '\0';
          strncpy(g_contacts[arrayIndex].phone, newPhone, sizeof(g_contacts[arrayIndex].phone));
          g_contacts[arrayIndex].phone[sizeof(g_contacts[arrayIndex].phone)-1] = '\0';
          g_contacts[arrayIndex].defined = true;
          ESP_LOGI(TAG, "Contact updated in SIM and memory at index %d", newIndex);
          sim900_send_sms(g_contacts[arrayIndex].phone, "Contact updated successfully.");
        } else {
          sim900_send_sms(g_contacts[senderContactIndex].phone, "Failed to update contact in SIM.");
        }
      }
    } else {
      ESP_LOGW(TAG, "Failed to parse update command: %s", command);
      sim900_send_sms(g_contacts[senderContactIndex].phone, "Update command parsing error.");
    }
}

static void process_delete_command(const char* command, int senderContactIndex) {
    int newIndex;
    
    // The command string starts with "delete contact ", so skip the first 15 characters.
    int n = sscanf(command + 15, "%d", &newIndex);
    
    if (n == 1) {
      if (newIndex >= 99 && newIndex <= 104 && newIndex!=senderContactIndex) {
        if (sim900_delete_contact(newIndex)) {
          int arrayIndex = newIndex - 99;
          // Update the global contact array with the new values.
          g_contacts[arrayIndex].password[0] = '\0';
          g_contacts[arrayIndex].phone[0] = '\0';
          g_contacts[arrayIndex].defined = false;
          ESP_LOGI(TAG, "Contact deleted in SIM and memory at index %d", newIndex);
          sim900_send_sms(g_contacts[arrayIndex].phone, "Contact updated successfully.");
        } else {
          sim900_send_sms(g_contacts[senderContactIndex].phone, "Failed to delete contact in SIM.");
        }
      }
    } else {
      ESP_LOGW(TAG, "Failed to parse delete command: %s", command);
      sim900_send_sms(g_contacts[senderContactIndex].phone, "Delete command parsing error.");
    }
}

// Processes the SMS command given the command text and contact index.
static void process_sms_command(const char* command, int contactIndex) {
    if (strncmp(command, "update contact", 7) == 0) {
        ESP_LOGI(TAG, "Updating contact!");
         process_update_command(command, contactIndex);
    } else if (strncmp(command, "delete contact", 7) == 0) {
        ESP_LOGI(TAG, "Updating contact!");
         process_delete_command(command, contactIndex);
    } else if (strcmp(command, "enable alerting") == 0) {
         g_alertingEnabled = true;
         ESP_LOGI(TAG, "Alerting enabled via SMS command.");
         sim900_send_sms(g_contacts[contactIndex].phone, "Alerting enabled.");
    } else if (strcmp(command, "disable alerting") == 0) {
         g_alertingEnabled = false;
         ESP_LOGI(TAG, "Alerting disabled via SMS command.");
         sim900_send_sms(g_contacts[contactIndex].phone, "Alerting disabled.");
    } else if (strcmp(command, "status") == 0) {
         ESP_LOGI(TAG, "Status command received.");
         sim900_send_full_status_sms_to(g_contacts[contactIndex].phone);
    } else {
         ESP_LOGW(TAG, "Unknown command received: '%s'", command);
    }
}

/////////////////////////////
// SMS Tasks
/////////////////////////////

void sim900_task_check_sms(void *pvParameters) {
  ESP_LOGI(TAG, "Starting SMS check task.");
  char response[512];
  char commandBuffer[64];
  for (;;) {
    ESP_LOGD(TAG, "Average temperature: %.2f", g_temperatureAvg);
    if (sendATCommand("AT+CMGL=\"REC UNREAD\"", response, sizeof(response), "OK", 5000)) {
      ESP_LOGD(TAG, "SMS list response: %s", response);
      int contactIndex = identify_contact_from_sms(response, commandBuffer, sizeof(commandBuffer));
      if (contactIndex >= 0) {
            ESP_LOGI(TAG, "SMS command from contact %d identified. Command: %s", contactIndex, commandBuffer);
            process_sms_command(commandBuffer, contactIndex);
      } else {
        ESP_LOGW(TAG, "Cannot identify contact from SMS or no contact at all. Sms: %s", response);
      }
      // Delete all received SMS.
      sendATCommand("AT+CMGDA=\"DEL ALL\"", response, sizeof(response), "OK", 5000);
    }
    vTaskDelay(pdMS_TO_TICKS(120000));
  }
}


