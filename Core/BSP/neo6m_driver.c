/**
 * @file neo6m_driver.c
 * @brief Concrete BSP Driver implementation for NEO-6M GPS.
 */

#include "neo6m_driver.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

extern UART_HandleTypeDef GPS_UART_HANDLE;
extern DMA_HandleTypeDef hdma_usart2_rx;

static uint8_t s_dma_rx_buf[NEO6M_DMA_BUF_SIZE];
static uint16_t s_last_dma_index = 0;

static char s_line_buf[NEO6M_NMEA_MAX_SENTENCE_LEN];
static uint16_t s_line_idx = 0;

static gps_data_t s_current_fix;
static bool s_has_fix = false;

static int hex_char_to_val(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static bool verify_nmea_checksum(const char *sentence)
{
    if (sentence == NULL || sentence[0] != '$')
    {
        return false;
    }

    uint8_t calculated_csum = 0;
    const char *p = sentence + 1; // Skip '$'

    while (*p && *p != '*' && *p != '\r' && *p != '\n')
    {
        calculated_csum ^= (uint8_t)(*p);
        p++;
    }

    if (*p != '*')
    {
        return false;
    }

    p++; // Skip '*'
    int h1 = hex_char_to_val(p[0]);
    int h2 = hex_char_to_val(p[1]);
    if (h1 < 0 || h2 < 0)
    {
        return false;
    }

    uint8_t expected_csum = (uint8_t)((h1 << 4) | h2);
    return (calculated_csum == expected_csum);
}

static double parse_nmea_coordinate(const char *coord_str, char dir)
{
    if (coord_str == NULL || strlen(coord_str) < 4)
    {
        return 0.0;
    }

    double raw = atof(coord_str);
    int degrees = (int)(raw / 100.0);
    double minutes = raw - (double)(degrees * 100);
    double decimal = (double)degrees + (minutes / 60.0);

    if (dir == 'S' || dir == 's' || dir == 'W' || dir == 'w')
    {
        decimal = -decimal;
    }

    return decimal;
}

static void parse_gpgga(char *sentence)
{
    /* Format: $GPGGA,time,lat,NS,lon,EW,quality,numSV,hdop,alt,M,sep,M,diffAge,diffStation*CS */
    char *tokens[15];
    int token_count = 0;
    char *p = sentence;

    while (*p && token_count < 15)
    {
        tokens[token_count++] = p;
        while (*p && *p != ',' && *p != '*')
        {
            p++;
        }
        if (*p == ',' || *p == '*')
        {
            *p = '\0';
            p++;
        }
    }

    if (token_count < 10)
    {
        return;
    }

    int quality = atoi(tokens[6]);
    if (quality > 0)
    {
        char ns = (tokens[3][0] != '\0') ? tokens[3][0] : 'N';
        char ew = (tokens[5][0] != '\0') ? tokens[5][0] : 'E';

        s_current_fix.latitude_deg = parse_nmea_coordinate(tokens[2], ns);
        s_current_fix.longitude_deg = parse_nmea_coordinate(tokens[4], ew);
        s_current_fix.satellites_visible = (uint8_t)atoi(tokens[7]);
        s_current_fix.hdop = (float)atof(tokens[8]);
        s_current_fix.altitude_m = (float)atof(tokens[9]);
        s_current_fix.fix_valid = true;
        s_current_fix.timestamp_ms = HAL_GetTick();
        s_has_fix = true;
    }
    else
    {
        s_current_fix.fix_valid = false;
        s_current_fix.satellites_visible = (uint8_t)atoi(tokens[7]);
        s_current_fix.timestamp_ms = HAL_GetTick();
    }
}

static void parse_gprmc(char *sentence)
{
    /* Format: $GPRMC,time,status,lat,NS,lon,EW,spd,cog,date,mv,mvEW,mode*CS */
    char *tokens[13];
    int token_count = 0;
    char *p = sentence;

    while (*p && token_count < 13)
    {
        tokens[token_count++] = p;
        while (*p && *p != ',' && *p != '*')
        {
            p++;
        }
        if (*p == ',' || *p == '*')
        {
            *p = '\0';
            p++;
        }
    }

    if (token_count < 9)
    {
        return;
    }

    char status = tokens[2][0];
    if (status == 'A') // Active / Valid fix
    {
        float speed_knots = (float)atof(tokens[7]);
        s_current_fix.ground_speed_m_s = speed_knots * 0.514444f;
        s_current_fix.course_deg = (float)atof(tokens[8]);
    }
}

static void process_nmea_sentence(char *sentence)
{
    if (!verify_nmea_checksum(sentence))
    {
        return;
    }

    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0)
    {
        parse_gpgga(sentence);
    }
    else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0)
    {
        parse_gprmc(sentence);
    }
}

void NEO6M_FeedBuffer(const uint8_t *buffer, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        char c = (char)buffer[i];

        if (c == '$')
        {
            s_line_idx = 0;
            s_line_buf[s_line_idx++] = c;
        }
        else if (s_line_idx > 0)
        {
            if (s_line_idx < (NEO6M_NMEA_MAX_SENTENCE_LEN - 1))
            {
                if (c == '\r' || c == '\n')
                {
                    s_line_buf[s_line_idx] = '\0';
                    process_nmea_sentence(s_line_buf);
                    s_line_idx = 0;
                }
                else
                {
                    s_line_buf[s_line_idx++] = c;
                }
            }
            else
            {
                /* Sentence exceeded buffer capacity -> discard */
                s_line_idx = 0;
            }
        }
    }
}

bool NEO6M_Init(void)
{
    memset(&s_current_fix, 0, sizeof(s_current_fix));
    s_line_idx = 0;
    s_has_fix = false;
    s_last_dma_index = 0;

    /* Start DMA circular receive on GPS UART */
    if (HAL_UART_Receive_DMA(&GPS_UART_HANDLE, s_dma_rx_buf, NEO6M_DMA_BUF_SIZE) != HAL_OK)
    {
        return false;
    }

    /* Enable USART IDLE line detection interrupt */
    __HAL_UART_ENABLE_IT(&GPS_UART_HANDLE, UART_IT_IDLE);

    return true;
}

void NEO6M_UART_IdleCallback(void)
{
    if (__HAL_UART_GET_FLAG(&GPS_UART_HANDLE, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&GPS_UART_HANDLE);

        /* Calculate current position in DMA ring buffer */
        uint16_t current_dma_index = NEO6M_DMA_BUF_SIZE - (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

        if (current_dma_index != s_last_dma_index)
        {
            if (current_dma_index > s_last_dma_index)
            {
                NEO6M_FeedBuffer(&s_dma_rx_buf[s_last_dma_index], current_dma_index - s_last_dma_index);
            }
            else
            {
                /* Buffer wrapped around */
                NEO6M_FeedBuffer(&s_dma_rx_buf[s_last_dma_index], NEO6M_DMA_BUF_SIZE - s_last_dma_index);
                if (current_dma_index > 0)
                {
                    NEO6M_FeedBuffer(&s_dma_rx_buf[0], current_dma_index);
                }
            }
            s_last_dma_index = current_dma_index;
        }
    }
}

bool NEO6M_GetFix(gps_data_t *out_data)
{
    if (out_data == NULL)
    {
        return false;
    }

    memcpy(out_data, &s_current_fix, sizeof(gps_data_t));
    return s_current_fix.fix_valid;
}

static const gps_interface_t s_neo6m_interface = {
    .init        = NEO6M_Init,
    .feed_buffer = NEO6M_FeedBuffer,
    .get_fix     = NEO6M_GetFix,
};

const gps_interface_t* NEO6M_GetInterface(void)
{
    return &s_neo6m_interface;
}
