#ifndef SIM900_H
#define SIM900_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

// Structure holding a SIM contact.
typedef struct {
    char phone[32];     // Phone number
    char password[64];  // Password (from contact name)
    bool defined;       // True if the contact exists
} Contact;

extern Contact g_contacts[5];  // Global array of 5 contacts

// Initializes and wakes up the SIM900 module.
void sim900_wakeup(void);

void validateStateOn(void);

// Checks if the SIM900 is ready (basic AT, signal quality, network registration).
bool sim900_is_ready(void);

//FIXME to be removed!!
// Reads contact info from the SIM phonebook entry at the specified index.
// The contact's name is used as the SMS password and the phone number as the recipient.
bool sim900_get_contact_info(int index, char* password, int pwdSize, char* phoneNumber, int phoneSize);

// Reads contacts from the SIM phonebook for indices 99 to 104.
int sim900_get_contacts(void);

// Sends an SMS message to the specified recipient.
bool sim900_send_sms(const char* recipient, const char* message);

//FIXME to be removed!!
// Sends a status SMS with SIM900 info only.
void sim900_send_status_sms(void);

// Sends a full status SMS (including SIM900, temperature and power info) to a specified recipient.
void sim900_send_full_status_sms_to(const char* recipient);

// Centralized function to send an alert to all defined contacts.
void sim900_send_alert(const char* msg);

// Sends a full status SMS including SIM900, temperature and power info.
void sim900_send_full_status_sms(void);

// Task to check for incoming SMS commands.
void sim900_task_check_sms(void *pvParameters);

// Task to monitor SIM900 status and reset it if unresponsive.
void sim900_task_monitor(void *pvParameters);

extern volatile bool g_sim900Ready;

// Global flag to control alerting (true by default).
extern volatile bool g_alertingEnabled;

#ifdef __cplusplus
}
#endif

#endif // SIM900_H
