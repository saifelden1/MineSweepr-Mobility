#include "main.h"
#include "mobility_config.h"
#include "sensors.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
static bool imu_ready;
static uint32_t imu_retry;
static uint8_t gps_byte;
static volatile uint16_t gps_head, gps_tail;
static volatile bool gps_overflow;
static char gps_ring[512];
static char sentence[128];
static unsigned sentence_len;
static bool gps_seen, gps_valid;
static uint32_t gps_time;
static double latitude, longitude, altitude;
static bool imu_init(void)
{
    uint8_t id=0, value=0;
    if (HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDRESS,0x75,1,&id,1,5)!=HAL_OK || id!=0x68) return false;
    /* Wake; +/-2g, +/-250 deg/s, DLPF and 50 Hz sampling. */
    const uint8_t regs[]={0x6B,0x1A,0x19,0x1B,0x1C};
    const uint8_t vals[]={0x01,0x03,0x13,0x00,0x00};
    for (unsigned i=0;i<sizeof(regs);i++) {
        value=vals[i];
        if (HAL_I2C_Mem_Write(&hi2c2,MPU6050_ADDRESS,regs[i],1,&value,1,5)!=HAL_OK) return false;
    }
    return true;
}
void sensors_init(void)
{
    imu_ready=imu_init(); imu_retry=HAL_GetTick();
    if (HAL_UART_Receive_IT(&huart2,&gps_byte,1)!=HAL_OK) Error_Handler();
}
void gps_rx_complete(void)
{
    uint16_t next=(gps_head+1U)%sizeof(gps_ring);
    if (next!=gps_tail) { gps_ring[gps_head]=(char)gps_byte; gps_head=next; }
    else gps_overflow=true;
    HAL_UART_Receive_IT(&huart2,&gps_byte,1);
}
void gps_uart_error(void)
{
    gps_overflow=true;
    HAL_UART_AbortReceive(&huart2);
    HAL_UART_Receive_IT(&huart2,&gps_byte,1);
}
static double coordinate(const char *number, const char *hemisphere)
{
    char *end;
    double raw=strtod(number,&end);
    if (end==number || *end || !isfinite(raw) || raw<0) return NAN;
    double degrees=floor(raw/100.0), minutes=raw-degrees*100.0;
    if (minutes>=60 || strlen(hemisphere)!=1) return NAN;
    double result=degrees+minutes/60.0;
    if (*hemisphere=='S' || *hemisphere=='W') return -result;
    return (*hemisphere=='N' || *hemisphere=='E') ? result : NAN;
}
static void parse_gga(char *line)
{
    if (line[0]!='$') return;
    char *star=strchr(line,'*');
    if (!star || strlen(star+1)!=2) return;
    uint8_t check=0;
    for (char *p=line+1;p<star;p++) check^=(uint8_t)*p;
    char *end;
    unsigned long expected=strtoul(star+1,&end,16);
    if (*end || end==star+1 || expected!=check) return;
    *star=0;
    char *field[16]; unsigned count=1; field[0]=line;
    for (char *p=line;*p;p++) if (*p==',') {
        *p=0; if (count<16) field[count++]=p+1;
    }
    if (count<13 || (strcmp(field[0],"$GPGGA") && strcmp(field[0],"$GNGGA"))) return;
    gps_seen=true; gps_time=HAL_GetTick(); gps_valid=false;
    long quality=strtol(field[6],&end,10);
    if (end==field[6] || *end || quality<1) return;
    latitude=coordinate(field[2],field[3]); longitude=coordinate(field[4],field[5]);
    altitude=strtod(field[9],&end);
    bool altitude_ok=end!=field[9] && !*end && isfinite(altitude) && !strcmp(field[10],"M");
    double geoid=strtod(field[11],&end);
    altitude_ok=altitude_ok && end!=field[11] && !*end && isfinite(geoid) && !strcmp(field[12],"M");
    altitude+=geoid;
    gps_valid=altitude_ok && isfinite(latitude) && isfinite(longitude) && fabs(latitude)<=90 && fabs(longitude)<=180;
}
void gps_poll(void)
{
    if (gps_overflow) {
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        gps_tail=gps_head; gps_overflow=false; __set_PRIMASK(mask);
        sentence_len=0; gps_seen=false;
    }
    while (gps_tail!=gps_head) {
        char ch=gps_ring[gps_tail]; gps_tail=(gps_tail+1U)%sizeof(gps_ring);
        if (ch=='$') { sentence_len=0; sentence[sentence_len++]=ch; }
        else if (ch=='\n') {
            sentence[sentence_len]=0; parse_gga(sentence); sentence_len=0;
        } else if (ch!='\r' && sentence_len>0) {
            if (sentence_len<sizeof(sentence)-1) sentence[sentence_len++]=ch;
            else sentence_len=0;
        }
    }
}
bool mobility_read_gps(sensor_msgs__msg__NavSatFix *msg)
{
    if (!gps_seen || HAL_GetTick()-gps_time>2500U) return false;
    msg->status.status=gps_valid ? sensor_msgs__msg__NavSatStatus__STATUS_FIX : sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;
    msg->status.service=sensor_msgs__msg__NavSatStatus__SERVICE_GPS;
    msg->latitude=gps_valid ? latitude : NAN;
    msg->longitude=gps_valid ? longitude : NAN;
    msg->altitude=gps_valid ? altitude : NAN;
    msg->position_covariance_type=sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN;
    return true;
}
bool mobility_read_imu(sensor_msgs__msg__Imu *msg)
{
    if (!imu_ready) {
        if (HAL_GetTick()-imu_retry<1000U) return false;
        imu_retry=HAL_GetTick(); imu_ready=imu_init();
        if (!imu_ready) return false;
    }
    uint8_t data[14];
    if (HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDRESS,0x3B,1,data,sizeof(data),5)!=HAL_OK) {
        imu_ready=false; imu_retry=HAL_GetTick(); return false;
    }
    double acc[3],gyro[3];
    for (unsigned i=0;i<3;i++) {
        int16_t a=(int16_t)((uint16_t)data[2*i]<<8 | data[2*i+1]);
        int16_t g=(int16_t)((uint16_t)data[8+2*i]<<8 | data[9+2*i]);
        acc[i]=a*9.80665/16384.0; gyro[i]=g*(3.141592653589793/180.0)/131.0;
    }
    msg->linear_acceleration.x=acc[0]; msg->linear_acceleration.y=acc[1]; msg->linear_acceleration.z=acc[2];
    msg->angular_velocity.x=gyro[0]; msg->angular_velocity.y=gyro[1]; msg->angular_velocity.z=gyro[2];
    msg->orientation.w=1.0; msg->orientation_covariance[0]=-1.0; /* No estimated orientation. */
    return true;
}

uint32_t gps_sample_time_ms(void) { return gps_time; }
