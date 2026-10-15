#ifndef MINESWEEPER_MOBILITY_DRIVE_H
#define MINESWEEPER_MOBILITY_DRIVE_H
#include <stdbool.h>
#include <stdint.h>

/* Encoder-free left/right commands in [-1,1]. The board adapter must drive
 * the actual Cytron PWM/DIR channels for all four motors; stop must zero
 * both drivers independently of ROS.
 * full_speed_mps is MEASURED at full PWM, not an encoder velocity estimate.
 */
typedef void (*mobility_write_fn)(float left, float right);
typedef struct {
    float track_width_m;
    float full_speed_mps;
    uint32_t timeout_ms;
    uint32_t last_command_ms;
    bool command_seen;
    mobility_write_fn write;
} mobility_drive_t;

bool mobility_drive_init(mobility_drive_t *drive, float track_width_m,
                         float full_speed_mps, uint32_t timeout_ms,
                         mobility_write_fn write);
bool mobility_drive_command(mobility_drive_t *drive, float linear_mps,
                            float angular_rps, uint32_t now_ms, bool estop);
/* Call from an independent >=100 Hz timer/interrupt, never only ROS spin. */
void mobility_drive_safety_tick(mobility_drive_t *drive, uint32_t now_ms, bool estop);
void mobility_drive_stop(mobility_drive_t *drive);
#endif
