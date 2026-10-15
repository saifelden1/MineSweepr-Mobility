#pragma once
#include <stdbool.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
void sensors_init(void);
void gps_poll(void);
bool mobility_read_imu(sensor_msgs__msg__Imu *msg);
bool mobility_read_gps(sensor_msgs__msg__NavSatFix *msg);
void gps_rx_complete(void);
void gps_uart_error(void);

uint32_t gps_sample_time_ms(void);
