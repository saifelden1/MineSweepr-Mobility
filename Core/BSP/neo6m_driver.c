/**
 * @file neo6m_driver.c
 * @brief u-blox NEO-6M GPS Receiver Driver Implementation.
 */

#include "neo6m_driver.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static UART_HandleTypeDef *s_huart = NULL;
static neo6m_gps_data_t s_latest_fix;
static char s_line_buf[128];
static uint8_t s_line_idx = 0;

void NEO6M_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    memset(&s_latest_fix, 0, sizeof(s_latest_fix));
    s_line_idx = 0;
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

bool NEO6M_GetLatestFix(neo6m_gps_data_t *data)
{
    if (data != NULL)
    {
        memcpy(data, &s_latest_fix, sizeof(neo6m_gps_data_t));
        return s_latest_fix.valid;
    }
    return false;
}
