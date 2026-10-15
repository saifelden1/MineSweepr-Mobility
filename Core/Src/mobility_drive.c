#include "mobility_drive.h"
#include <math.h>
#include <stddef.h>

void mobility_drive_stop(mobility_drive_t *drive)
{
    if (drive == NULL) return;
    drive->command_seen = false;
    if (drive->write != NULL) drive->write(0.0f, 0.0f);
}

bool mobility_drive_init(mobility_drive_t *drive, float track_width_m,
                         float full_speed_mps, uint32_t timeout_ms,
                         mobility_write_fn write)
{
    if (drive == NULL) return false;
    *drive = (mobility_drive_t){.write = write};
    mobility_drive_stop(drive);
    if (write == NULL || !isfinite(track_width_m) || track_width_m <= 0.0f ||
        !isfinite(full_speed_mps) || full_speed_mps <= 0.0f || timeout_ms == 0) return false;
    drive->track_width_m = track_width_m;
    drive->full_speed_mps = full_speed_mps;
    drive->timeout_ms = timeout_ms;
    return true;
}

bool mobility_drive_command(mobility_drive_t *drive, float linear_mps,
                            float angular_rps, uint32_t now_ms, bool estop)
{
    if (drive == NULL) return false;
    if (estop || drive->timeout_ms == 0 || !isfinite(linear_mps) || !isfinite(angular_rps)) {
        mobility_drive_stop(drive);
        return false;
    }
    float left = (linear_mps - angular_rps * drive->track_width_m * 0.5f) / drive->full_speed_mps;
    float right = (linear_mps + angular_rps * drive->track_width_m * 0.5f) / drive->full_speed_mps;
    if (!isfinite(left) || !isfinite(right)) {
        mobility_drive_stop(drive);
        return false;
    }
    float scale = fmaxf(1.0f, fmaxf(fabsf(left), fabsf(right)));
    drive->last_command_ms = now_ms;
    drive->command_seen = true;
    drive->write(left / scale, right / scale);
    return true;
}

void mobility_drive_safety_tick(mobility_drive_t *drive, uint32_t now_ms, bool estop)
{
    if (drive == NULL) return;
    if (estop || !drive->command_seen ||
        (uint32_t)(now_ms - drive->last_command_ms) >= drive->timeout_ms) {
        mobility_drive_stop(drive);
    }
}
