#pragma once
#include <stdbool.h>
void mobility_write_cytron(float left, float right);
bool mobility_estop_asserted(void);
void motor_safety_tick(void);
void drive_init(void);
bool ros_entities_create(void);
void ros_entities_destroy(void);
void ros_spin(void);
void drive_stop(void);
