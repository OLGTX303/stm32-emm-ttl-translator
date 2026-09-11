#ifndef TRANSLATOR_H
#define TRANSLATOR_H
#include <stdint.h>
#include <stdbool.h>
#include "motor_config.h"

/* Platform-independent production core. IRQs only operate UART rings. */
enum { TR_OK, TR_FRAME, TR_BUSY, TR_QUEUE, TR_RANGE, TR_DISABLED,
       TR_TIMEOUT, TR_MOTOR, TR_STALE, TR_DEADLINE, TR_STALL, TR_CONFIG,
       TR_UART, TR_UNDERRUN };
typedef struct {
    int32_t target;
    int16_t end_rpm, max_rpm, accel;
    uint8_t current, home, mask;
    uint32_t group;
} TrSegment;
typedef struct {
    float v0, v1, vc, a, ta, tv, td, duration, distance;
} TrProfile;
typedef struct {
    TrSegment queue[TR_QUEUE_SIZE];
    uint8_t head, count, active, enabled, configured, fault, holding;
    uint8_t flags, current, valid, no_temperature;
    int8_t temperature;
    uint16_t voltage, commands;
    uint16_t fw_version;
    uint8_t hw_series, hw_type, hw_version;
    int32_t position, start_position, last_sent, stationary_position;
    uint32_t pos_us, flags_us, temp_us, voltage_us, start_us, stable_us;
    uint32_t finish_us, last_sample_us, cruise_until_us;
    float carry_rpm;
    TrProfile profile;
} TrMotor;
typedef struct {
    uint32_t host_frames, bad_frames, motor_frames, timeouts, deadline_faults;
    uint32_t max_tick_late_us, stream_packets, max_bus_transaction_us;
    uint8_t last_error, unsupported_temperature_mask, last_error_length;
    uint8_t last_error_frame[16];
} TrDiagnostics;
extern TrMotor tr_motor[MOTOR_COUNT];
extern TrDiagnostics tr_diag;
/* Copy a complete packet or return false; never block.
 * bus_idle means the final stop bit has left, not just TXE. */
bool tr_bus_write(const uint8_t *data, uint16_t length);
bool tr_bus_idle(void);
bool tr_host_write(const uint8_t *data, uint16_t length);
void tr_init(uint32_t now_us);
void tr_poll(uint32_t now_us);
void tr_host_byte(uint8_t byte, uint32_t now_us);
void tr_motor_byte(uint8_t byte, uint32_t now_us);
void tr_uart_fault(void);
uint8_t tr_crc8(const uint8_t *data, uint16_t length);
bool tr_profile(TrProfile *p, int64_t distance, float start_rpm,
                int16_t end_rpm, int16_t max_rpm, int16_t accel);
float tr_profile_position(const TrProfile *p, float seconds);
int64_t tr_counts_to_pulses(uint8_t motor, int32_t counts);
#endif
