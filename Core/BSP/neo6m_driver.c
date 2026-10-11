/**
 * @file neo6m_driver.c
 * @brief u-blox NEO-6M GPS Receiver Driver Implementation.
 */

#include "neo6m_driver.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static UART_HandleTypeDef *s_huart = NULL;
static DMA_HandleTypeDef s_hdma_gps_rx;
static neo6m_gps_data_t s_latest_fix;
static char s_line_buf[128];
static uint8_t s_line_idx = 0;

static uint8_t s_gps_dma_buf[NEO6M_DMA_BUF_SIZE] __attribute__((aligned(4)));
static volatile uint16_t s_gps_dma_tail = 0;

bool NEO6M_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    memset(&s_latest_fix, 0, sizeof(s_latest_fix));
    memset(s_gps_dma_buf, 0, sizeof(s_gps_dma_buf));
    s_line_idx = 0;
    s_gps_dma_tail = 0;

    if (s_huart == NULL)
    {
        return false;
    }

    /* Configure DMA1_Stream5 Channel 4 for USART2 circular RX if not already linked */
    if (s_huart->hdmarx == NULL)
    {
        __HAL_RCC_DMA1_CLK_ENABLE();
        s_hdma_gps_rx.Instance = DMA1_Stream5;
        s_hdma_gps_rx.Init.Channel = DMA_CHANNEL_4;
        s_hdma_gps_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        s_hdma_gps_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        s_hdma_gps_rx.Init.MemInc = DMA_MINC_ENABLE;
        s_hdma_gps_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        s_hdma_gps_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        s_hdma_gps_rx.Init.Mode = DMA_CIRCULAR;
        s_hdma_gps_rx.Init.Priority = DMA_PRIORITY_LOW;
        s_hdma_gps_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&s_hdma_gps_rx);
        __HAL_LINKDMA(s_huart, hdmarx, s_hdma_gps_rx);

        HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    }

    HAL_UART_Receive_DMA(s_huart, s_gps_dma_buf, NEO6M_DMA_BUF_SIZE);
    return true;
}

static double convert_nmea_deg(const char *val, char hemi)
{
    if (val == NULL || strlen(val) < 4) return 0.0;
    double raw = atof(val);
    int degrees = (int)(raw / 100.0);
    double minutes = raw - (degrees * 100.0);
    double dec = (double)degrees + (minutes / 60.0);
    if (hemi == 'S' || hemi == 'W') dec = -dec;
    return dec;
}

static void parse_gpgga_sentence(char *line)
{
    /* Format: $GPGGA,time,lat,N/S,lon,E/W,quality,sats,hdop,alt,M,geoid,M,age,station*cs */
    char *star = strchr(line, '*');
    if (star == NULL)
    {
        return; /* Missing NMEA checksum delimiter */
    }

    /* Compute XOR checksum of all characters between '$' and '*' */
    uint8_t computed_cs = 0;
    char *p_cs = line;
    if (*p_cs == '$')
    {
        p_cs++;
    }
    while (p_cs < star)
    {
        computed_cs ^= (uint8_t)(*p_cs);
        p_cs++;
    }

    /* Verify 2-character hex checksum following '*' */
    if (star[1] == '\0' || star[2] == '\0')
    {
        return;
    }
    char cs_hex[3] = { star[1], star[2], '\0' };
    char *endptr = NULL;
    uint8_t expected_cs = (uint8_t)strtoul(cs_hex, &endptr, 16);
    if (endptr != &cs_hex[2] || computed_cs != expected_cs)
    {
        return; /* Checksum mismatch: reject corrupted sentence */
    }

    /* Null-terminate sentence body at '*' so tokens do not contain checksum */
    *star = '\0';

    char *tokens[15];
    int token_count = 0;
    char *p = line;

    while (p != NULL && token_count < 15)
    {
        tokens[token_count++] = p;
        p = strchr(p, ',');
        if (p != NULL)
        {
            *p = '\0';
            p++;
        }
    }

    if (token_count >= 10 && (strcmp(tokens[0], "$GPGGA") == 0 || strcmp(tokens[0], "$GNGGA") == 0))
    {
        int quality = atoi(tokens[6]);
        if (quality > 0)
        {
            s_latest_fix.latitude = convert_nmea_deg(tokens[2], tokens[3][0]);
            s_latest_fix.longitude = convert_nmea_deg(tokens[4], tokens[5][0]);
            s_latest_fix.fix_quality = (int8_t)quality;
            s_latest_fix.satellites = (uint8_t)atoi(tokens[7]);
            s_latest_fix.hdop = (float)atof(tokens[8]);
            s_latest_fix.altitude = atof(tokens[9]);
            s_latest_fix.valid = true;
        }
    }
}

void NEO6M_ProcessByte(uint8_t c)
{
    if (c == '$')
    {
        s_line_idx = 0;
        s_line_buf[s_line_idx++] = (char)c;
    }
    else if (c == '\r' || c == '\n')
    {
        if (s_line_idx > 6)
        {
            s_line_buf[s_line_idx] = '\0';
            parse_gpgga_sentence(s_line_buf);
        }
        s_line_idx = 0;
    }
    else
    {
        if (s_line_idx < sizeof(s_line_buf) - 1)
        {
            s_line_buf[s_line_idx++] = (char)c;
        }
        else
        {
            s_line_idx = 0;
        }
    }
}

void NEO6M_ProcessDmaBuffer(void)
{
    if (s_huart == NULL || s_huart->hdmarx == NULL)
    {
        return;
    }

    uint16_t head = NEO6M_DMA_BUF_SIZE - (uint16_t)__HAL_DMA_GET_COUNTER(s_huart->hdmarx);
    while (s_gps_dma_tail != head)
    {
        uint8_t byte = s_gps_dma_buf[s_gps_dma_tail];
        s_gps_dma_tail = (s_gps_dma_tail + 1) % NEO6M_DMA_BUF_SIZE;
        NEO6M_ProcessByte(byte);
    }
}

bool NEO6M_GetLatestFix(neo6m_gps_data_t *data)
{
    if (data != NULL)
    {
        memcpy(data, &s_latest_fix, sizeof(neo6m_gps_data_t));
        return s_latest_fix.valid;
    }
    return false;
}

bool NEO6M_GetData(neo6m_gps_data_t *data)
{
    NEO6M_ProcessDmaBuffer();
    return NEO6M_GetLatestFix(data);
}
