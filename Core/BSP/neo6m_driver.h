/**
 * @file neo6m_driver.h
 * @brief Concrete BSP Driver for u-blox NEO-6M GPS Receiver over USART2.
 *
 * Implements gps_interface_t with DMA circular reception, IDLE interrupt handling,
 * and robust non-blocking NMEA sentence parsing (GPGGA / GPRMC) with strict XOR checksum validation.
 */

#ifndef NEO6M_DRIVER_H
#define NEO6M_DRIVER_H

#include "interfaces/gps_interface.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NEO6M_DMA_BUF_SIZE          (512U)
#define NEO6M_NMEA_MAX_SENTENCE_LEN (96U)

/**
 * @brief Initialize NEO-6M hardware driver (USART2 DMA circular receive).
 * @return true on success.
 */
bool NEO6M_Init(void);

/**
 * @brief Feed raw serial byte buffer into the internal NMEA stream parser.
 * @param buffer Pointer to received byte array.
 * @param len Number of bytes in buffer.
 */
void NEO6M_FeedBuffer(const uint8_t *buffer, uint16_t len);

/**
 * @brief Retrieve the most recent valid GPS fix data.
 * @param out_data Pointer to gps_data_t destination structure.
 * @return true if fix is currently valid, false otherwise.
 */
bool NEO6M_GetFix(gps_data_t *out_data);

/**
 * @brief Callback invoked on USART2 IDLE line detection interrupt.
 */
void NEO6M_UART_IdleCallback(void);

/**
 * @brief Get the gps_interface_t function pointer table.
 * @return Pointer to gps_interface_t instance.
 */
const gps_interface_t* NEO6M_GetInterface(void);

#ifdef __cplusplus
}
#endif

#endif /* NEO6M_DRIVER_H */
