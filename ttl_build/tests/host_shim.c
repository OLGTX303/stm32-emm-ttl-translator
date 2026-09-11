#include "translator.h"
#include <string.h>

static uint8_t bus_out[256], host_out[4096];
static uint16_t bus_size, host_size;
static uint32_t clock_us, tx_end;
bool tr_bus_write(const uint8_t *p, uint16_t n)
{
    if(bus_size || n>sizeof(bus_out)) return false;
    memcpy(bus_out,p,n); bus_size=n;
    tx_end=clock_us+((uint32_t)n*10000000UL+MOTOR_DEFAULT_BAUD-1)/MOTOR_DEFAULT_BAUD;
    return true;
}
bool tr_bus_idle(void) { return (int32_t)(clock_us-tx_end)>=0; }
bool tr_host_write(const uint8_t *p, uint16_t n)
{
    if(host_size+n>sizeof(host_out)) return false;
    memcpy(host_out+host_size,p,n); host_size+=n; return true;
}
void test_reset(void)
{
    bus_size=host_size=0; clock_us=tx_end=0; tr_init(0);
}
void test_poll(uint32_t t) { clock_us=t; tr_poll(t); }
void test_host(uint8_t b, uint32_t t) { clock_us=t; tr_host_byte(b,t); }
void test_motor(uint8_t b, uint32_t t) { clock_us=t; tr_motor_byte(b,t); }
uint16_t test_bus(uint8_t *out) { uint16_t n=bus_size; memcpy(out,bus_out,n); bus_size=0; return n; }
uint16_t test_host_out(uint8_t *out) { uint16_t n=host_size; memcpy(out,host_out,n); host_size=0; return n; }
uint32_t test_tx_end(void) { return tx_end; }
uint8_t test_fault(void) { return tr_diag.last_error; }
uint8_t test_count(uint8_t i) { return tr_motor[i].count; }
uint8_t test_active(uint8_t i) { return tr_motor[i].active; }
int32_t test_position(uint8_t i) { return tr_motor[i].position; }
uint32_t test_streams(void) { return tr_diag.stream_packets; }
uint32_t test_start(uint8_t i) { return tr_motor[i].start_us; }
float test_carry(uint8_t i) { return tr_motor[i].carry_rpm; }
uint8_t test_enabled(uint8_t i) { return tr_motor[i].enabled; }
uint16_t test_commands(uint8_t i) { return tr_motor[i].commands; }
uint32_t test_late(void) { return tr_diag.max_tick_late_us; }
int32_t test_target(uint8_t i) { return tr_motor[i].last_sent; }
uint32_t test_finish(uint8_t i) { return tr_motor[i].finish_us; }
void test_set_counter(uint8_t i, uint16_t count) { tr_motor[i].commands=count; }
void test_advance_clock(uint32_t t)
{
    bus_size=host_size=0; clock_us=tx_end=t; tr_init(t);
}
