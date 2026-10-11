/**
 * @file neo6m_driver.h
 * @brief u-blox NEO-6M GPS Receiver Driver for STM32_Mobility.
 */

#ifndef NEO6M_DRIVER_H
#define NEO6M_DRIVER_H

#include "mobility_pin_config.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double  latitude;
    double  longitude;
    double  altitude;
    int8_t  fix_quality;
    uint8_t satellites;
    float   hdop;
    bool    valid;
} neo6m_gps_data_t;

/**
 * @brief Initialize NEO-6M GPS receiver on USART2.
 * @param huart Pointer to UART_HandleTypeDef (e.g. &huart2).
 */
void NEO6M_Init(UART_HandleTypeDef *huart);

/**
 * @brief Feed incoming UART character into non-blocking NMEA parser.
 * @param c Received ASCII character.
 */
void NEO6M_ProcessByte(uint8_t c);

/**
 * @brief Get latest decoded GPS fix.
 * @param data Output data structure.
 * @return true if fix is valid.
 */
bool NEO6M_GetLatestFix(neo6m_gps_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* NEO6M_DRIVER_H */
