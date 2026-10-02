/**
 * @file microros_client.c
 * @brief micro-ROS Client & Topic Publisher/Subscriber Implementation.
 */

#include "microros_client.h"
#include "microros_transport.h"
#include <string.h>

#define SYNC_BYTE_1             (0xAAU)
#define SYNC_BYTE_2             (0x55U)

typedef enum {
    PARSE_STATE_SYNC1 = 0,
    PARSE_STATE_SYNC2,
    PARSE_STATE_TOPIC_ID,
    PARSE_STATE_LEN_L,
    PARSE_STATE_LEN_H,
    PARSE_STATE_PAYLOAD,
    PARSE_STATE_CRC_L,
    PARSE_STATE_CRC_H,
} rx_parse_state_t;

static cmd_vel_callback_t s_cmd_vel_callback = NULL;
static microros_stats_t s_stats;

static rx_parse_state_t s_rx_state = PARSE_STATE_SYNC1;
static uint8_t s_rx_topic_id = 0;
static uint16_t s_rx_payload_len = 0;
static uint16_t s_rx_payload_idx = 0;
static uint8_t s_rx_payload_buf[256];
static uint16_t s_rx_received_crc = 0;

static uint16_t compute_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

bool MicroROS_Client_Init(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
    s_rx_state = PARSE_STATE_SYNC1;
    s_cmd_vel_callback = NULL;
    return MicroROS_Transport_Init();
}

void MicroROS_Client_RegisterCmdVelCallback(cmd_vel_callback_t cb)
{
    s_cmd_vel_callback = cb;
}

static bool transmit_framed_message(uint8_t topic_id, const uint8_t *payload, uint16_t len)
{
    static uint8_t s_tx_buf[512];
    if (len > sizeof(s_tx_buf) - 7) {
        return false;
    }

    s_tx_buf[0] = SYNC_BYTE_1;
    s_tx_buf[1] = SYNC_BYTE_2;
    s_tx_buf[2] = topic_id;
    s_tx_buf[3] = (uint8_t)(len & 0xFF);
    s_tx_buf[4] = (uint8_t)((len >> 8) & 0xFF);

    if (len > 0 && payload != NULL) {
        memcpy(&s_tx_buf[5], payload, len);
    }

    uint16_t crc = compute_crc16(&s_tx_buf[2], (uint16_t)(3 + len));
    s_tx_buf[5 + len] = (uint8_t)(crc & 0xFF);
    s_tx_buf[6 + len] = (uint8_t)((crc >> 8) & 0xFF);

    size_t total_len = (size_t)(7 + len);
    size_t written = MicroROS_Transport_Write(s_tx_buf, total_len, 20);
    return (written == total_len);
}

bool MicroROS_Client_PublishOdom(const odometry_msg_t *odom)
{
    if (odom == NULL) {
        return false;
    }

    bool ok = transmit_framed_message(TOPIC_ID_ODOM, (const uint8_t*)odom, sizeof(odometry_msg_t));
    if (ok) {
        s_stats.odom_published_count++;
    }
    return ok;
}

bool MicroROS_Client_PublishImu(const imu_msg_t *imu)
{
    if (imu == NULL) {
        return false;
    }

    bool ok = transmit_framed_message(TOPIC_ID_IMU, (const uint8_t*)imu, sizeof(imu_msg_t));
    if (ok) {
        s_stats.imu_published_count++;
    }
    return ok;
}

bool MicroROS_Client_PublishGps(const navsatfix_msg_t *gps)
{
    if (gps == NULL) {
        return false;
    }

    bool ok = transmit_framed_message(TOPIC_ID_GPS, (const uint8_t*)gps, sizeof(navsatfix_msg_t));
    if (ok) {
        s_stats.gps_published_count++;
    }
    return ok;
}

bool MicroROS_Client_PublishHeartbeat(const bool_msg_t *heartbeat)
{
    if (heartbeat == NULL) {
        return false;
    }

    bool ok = transmit_framed_message(TOPIC_ID_HEARTBEAT, (const uint8_t*)heartbeat, sizeof(bool_msg_t));
    if (ok) {
        s_stats.heartbeat_published_count++;
    }
    return ok;
}

static void process_received_frame(uint8_t topic_id, const uint8_t *payload, uint16_t len)
{
    if (topic_id == TOPIC_ID_CMD_VEL) {
        if (len >= sizeof(twist_msg_t)) {
            twist_msg_t twist;
            memcpy(&twist, payload, sizeof(twist_msg_t));
            s_stats.cmd_vel_received_count++;
            s_stats.agent_connected = true;

            if (s_cmd_vel_callback != NULL) {
                s_cmd_vel_callback(&twist);
            }
        } else {
            s_stats.framing_error_count++;
        }
    }
}

void MicroROS_Client_SpinSome(uint32_t timeout_ms)
{
    (void)timeout_ms;
    uint8_t byte;

    while (MicroROS_Transport_Read(&byte, 1, 0) > 0) {
        switch (s_rx_state) {
            case PARSE_STATE_SYNC1:
                if (byte == SYNC_BYTE_1) {
                    s_rx_state = PARSE_STATE_SYNC2;
                }
                break;

            case PARSE_STATE_SYNC2:
                if (byte == SYNC_BYTE_2) {
                    s_rx_state = PARSE_STATE_TOPIC_ID;
                } else if (byte != SYNC_BYTE_1) {
                    s_rx_state = PARSE_STATE_SYNC1;
                }
                break;

            case PARSE_STATE_TOPIC_ID:
                s_rx_topic_id = byte;
                s_rx_state = PARSE_STATE_LEN_L;
                break;

            case PARSE_STATE_LEN_L:
                s_rx_payload_len = (uint16_t)byte;
                s_rx_state = PARSE_STATE_LEN_H;
                break;

            case PARSE_STATE_LEN_H:
                s_rx_payload_len |= (uint16_t)(byte << 8);
                s_rx_payload_idx = 0;
                if (s_rx_payload_len > sizeof(s_rx_payload_buf)) {
                    s_stats.framing_error_count++;
                    s_rx_state = PARSE_STATE_SYNC1;
                } else if (s_rx_payload_len == 0) {
                    s_rx_state = PARSE_STATE_CRC_L;
                } else {
                    s_rx_state = PARSE_STATE_PAYLOAD;
                }
                break;

            case PARSE_STATE_PAYLOAD:
                s_rx_payload_buf[s_rx_payload_idx++] = byte;
                if (s_rx_payload_idx >= s_rx_payload_len) {
                    s_rx_state = PARSE_STATE_CRC_L;
                }
                break;

            case PARSE_STATE_CRC_L:
                s_rx_received_crc = (uint16_t)byte;
                s_rx_state = PARSE_STATE_CRC_H;
                break;

            case PARSE_STATE_CRC_H:
                s_rx_received_crc |= (uint16_t)(byte << 8);
                {
                    uint8_t header_buf[3];
                    header_buf[0] = s_rx_topic_id;
                    header_buf[1] = (uint8_t)(s_rx_payload_len & 0xFF);
                    header_buf[2] = (uint8_t)((s_rx_payload_len >> 8) & 0xFF);

                    uint16_t crc = 0xFFFF;
                    /* Compute CRC across header (3 bytes) and payload */
                    for (int i = 0; i < 3; i++) {
                        crc ^= header_buf[i];
                        for (int j = 0; j < 8; j++) {
                            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
                        }
                    }
                    for (uint16_t i = 0; i < s_rx_payload_len; i++) {
                        crc ^= s_rx_payload_buf[i];
                        for (int j = 0; j < 8; j++) {
                            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
                        }
                    }

                    if (crc == s_rx_received_crc) {
                        process_received_frame(s_rx_topic_id, s_rx_payload_buf, s_rx_payload_len);
                    } else {
                        s_stats.framing_error_count++;
                    }
                }
                s_rx_state = PARSE_STATE_SYNC1;
                break;

            default:
                s_rx_state = PARSE_STATE_SYNC1;
                break;
        }
    }
}

void MicroROS_Client_GetStats(microros_stats_t *out_stats)
{
    if (out_stats != NULL) {
        *out_stats = s_stats;
    }
}
