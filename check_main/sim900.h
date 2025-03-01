#ifndef SIM900_H
#define SIM900_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes and wakes up the SIM900 module.
void sim900_wakeup(void);

// Checks if the SIM900 is ready (basic AT, signal quality, network registration).
bool sim900_is_ready(void);

// Reads contact info from the SIM phonebook entry at the specified index.
// The contact's name is used as the SMS password and the phone number as the recipient.
bool sim900_get_contact_info(int index, char* password, int pwdSize, char* phoneNumber, int phoneSize);

// Sends an SMS message to the specified recipient.
bool sim900_send_sms(const char* recipient, const char* message);

// Sends a status SMS with SIM900 info only.
void sim900_send_status_sms(void);

// Sends a full status SMS including SIM900, temperature and power info.
void sim900_send_full_status_sms(void);

// Task to check for incoming SMS commands.
void sim900_task_check_sms(void *pvParameters);

// Task to monitor SIM900 status and reset it if unresponsive.
void sim900_task_monitor(void *pvParameters);

// Global variables for SMS command authentication.
extern char g_smsRecipient[32];
extern char g_smsPassword[64];

extern volatile bool g_sim900Ready;

// Global flag to control alerting (true by default).
extern volatile bool g_alertingEnabled;

#ifdef __cplusplus
}
#endif

#endif // SIM900_H
