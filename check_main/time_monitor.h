#ifndef TIME_MONITOR_H
#define TIME_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

// Global string holding the startup network time (as obtained from SIM900).
extern char g_startTime[32];

// Initializes time monitoring by reading the SIM900 network time.
// Returns true if successful.
bool time_monitor_init(void);

#ifdef __cplusplus
}
#endif

#endif // TIME_MONITOR_H
