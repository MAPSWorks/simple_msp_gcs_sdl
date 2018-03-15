#include "msp_protocol.h"

#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include "../serial_setup/serial_setup.h"
#include "../rpi-udp-stream-client/common_util/common_util.h"

static att_t att = {0, };
static alt_t alt = {0, };
static imu_t imu = {0, };
static msp_status_t msp_status = {0, };
static int16_t debug[4] = {0, };

static int16_t roll_input = 0;
static int16_t pitch_input = 0;
static int16_t yaw_input = 0;
static int16_t throttle_input = 0;
static int16_t althold_switch_input = 0;

static uint8_t msp_arm_request = 0;
static uint8_t msp_disarm_request = 0;
static uint8_t msp_acc_calib_request = 0;
static uint8_t msp_mag_calib_request = 0;
static uint8_t msp_eeprom_write_request = 0;

enum
{
    WAIT_$ = 0,
    WAIT_M,
    WAIT_ARROW,
    WAIT_BUFSIZE,
    WAIT_CMD,
    WAIT_DATA,

    MAXBUF_SIZE = 64,
};

enum
{
    RC_MAX = 1900,
    RC_MIN = 1100,
    RC_MID = 1500,

    rcRoll = 0,
    rcPitch,
    rcYaw,
    rcThrottle,
    rcAux1,
    rcAux2,
    rcAux3,
    rcAux4,
};

static void msp_write_cmd(uint8_t cmd)
{
    serial_write('$');
    serial_write('M');
    serial_write('<');
    serial_write(0);
    serial_write(cmd);
    serial_write(cmd);
}

static void msp_write_buf(uint8_t cmd, uint8_t* buf, uint8_t size)
{
    uint8_t checksum = 0;
    checksum = cmd^size;

    serial_write('$');
    serial_write('M');
    serial_write('<');
    serial_write(size);
    serial_write(cmd);
    for(uint8_t i = 0; i < size; i++)
    {
        serial_write(buf[i]);
        checksum ^= buf[i];
    }
    serial_write(checksum);
}

static int msp_parse_cmd(uint8_t* received_cmd, uint8_t* received_size, uint8_t* received_buf)
{
    int retval = 0;

    uint8_t c;

    static uint8_t cmd;
    static uint8_t dataSize;
    static uint8_t dataBuf[MAXBUF_SIZE];
    static uint8_t dataBufCount = 0;

    static uint8_t checkSum = 0;
    static uint8_t state = WAIT_$;

    if(serial_available())
    {
        c = serial_read();
        switch(state)
        {
            case WAIT_$ :
                state = (c == '$') ? WAIT_M : WAIT_$;
                break;
            case WAIT_M :
                state = (c == 'M') ? WAIT_ARROW : WAIT_$;
                break;
            case WAIT_ARROW :
                state = (c == '>') ? WAIT_BUFSIZE : WAIT_$;
                break;
            case WAIT_BUFSIZE :
                if(c > MAXBUF_SIZE)
                {
                    state = WAIT_$; //buffer size error
                }
                else
                {
                    dataSize = c;
                    checkSum ^= c;
                    state = WAIT_CMD;
                }
                break;
            case WAIT_CMD :
                cmd = c;
                checkSum ^= c;
                state = WAIT_DATA;
                break;
            case WAIT_DATA :
                if(dataBufCount < dataSize)
                {
                    dataBuf[dataBufCount]= c;
                    dataBufCount++;
                    checkSum ^= c;
                }
                else
                {
                    if(checkSum == c) //All data set Successfully received
                    {
                        //copy buffer to class's buffer
                        *received_cmd = cmd;
                        *received_size = dataSize;
                        uint8_t i;
                        for(i = 0; i < dataSize; i++)
                        {
                            received_buf[i] = dataBuf[i];
                        }
                        state = WAIT_$;

                        retval = 1;
                    }
                    else // error appeared in checksum
                    {
                        state = WAIT_$;
                    }
                    //initialize all static variables
                    dataBufCount = 0;
                    checkSum = 0;
                }

                break;
        }
    }

    return retval;
}

static void msp_send_rc()
{
    uint16_t rc16[8];
    uint8_t rc8[16];
    rc16[rcRoll] = RC_MID + roll_input;
    rc16[rcPitch] = RC_MID + pitch_input;
    rc16[rcYaw] = RC_MID + yaw_input;
    rc16[rcThrottle] = RC_MIN + throttle_input;
    rc16[rcAux1] = RC_MIN + althold_switch_input;
    rc16[rcAux2] = RC_MIN;
    rc16[rcAux3] = RC_MIN;
    rc16[rcAux4] = RC_MIN;

    for (uint8_t i = 0; i < 8; i++)
    {
        rc8[2 * i] = rc16[i] & 0xff;
        rc8[2 * i + 1] = (rc16[i] >> 8) & 0xff;
    }

    msp_write_buf(MSP_SET_RAW_RC, rc8, 16);
}

static void* send_msp_thread(void* arg)
{
    static uint8_t cmd_state = 0;

    while(!is_quit())
    {
        switch(cmd_state)
        {
            case 0:
                msp_send_rc();
                cmd_state++;
                break;
            case 1:
                if(msp_arm_request)
                {
                    msp_write_cmd(MSP_ARM);
                    msp_arm_request = 0;
                }
                if(msp_disarm_request)
                {
                    msp_write_cmd(MSP_DISARM);
                    msp_disarm_request = 0;
                }
                cmd_state++;
                break;
            case 2:
                msp_write_cmd(MSP_ATTITUDE);
                cmd_state++;
                break;
            case 3:
                msp_write_cmd(MSP_ALTITUDE);
                cmd_state++;
                break;
            case 4:
                msp_write_cmd(MSP_RAW_IMU);
                cmd_state++;
                break;
            case 5:
                msp_write_cmd(MSP_DEBUG);
                cmd_state++;
                break;
            case 6:
                if(msp_acc_calib_request)
                {
                    msp_write_cmd(MSP_ACC_CALIBRATION);
                    msp_acc_calib_request = 0;
                }
                if(msp_mag_calib_request)
                {
                    msp_write_cmd(MSP_MAG_CALIBRATION);
                    msp_mag_calib_request = 0;
                }
                if(msp_eeprom_write_request)
                {
                    msp_write_cmd(MSP_EEPROM_WRITE);
                    msp_eeprom_write_request = 0;
                }
                cmd_state++;
                break;
            case 7:
                msp_write_cmd(MSP_STATUS);
                cmd_state++;
            default :
                cmd_state = 0;
                break;
        }

        usleep(6000);
    }
    pthread_exit((void*)0);
}

static void* received_msp_thread(void* arg)
{
    uint8_t received_cmd;
    uint8_t received_size;
    uint8_t received_buf[MAXBUF_SIZE];

    while(!is_quit())
    {
        if(msp_parse_cmd(&received_cmd, &received_size, received_buf))
        {
            switch(received_cmd)
            {
                case MSP_STATUS:
                    memcpy((uint8_t*)&msp_status, received_buf, received_size);
                    break;
                case MSP_ATTITUDE:
                    memcpy((uint8_t*)&att, received_buf, received_size);
                    break;
                case MSP_ALTITUDE:
                    memcpy((uint8_t*)&alt, received_buf, received_size);
                    break;
                case MSP_RAW_IMU:
                    memcpy((uint8_t*)&imu, received_buf, received_size);
                    break;
                case MSP_DEBUG:
                    memcpy((uint8_t*)debug, received_buf, received_size);
                    break;
                default :
                    break;
            }
        }
        usleep(100);
    }
    pthread_exit((void*)0);
}

void msp_init()
{
    pthread_t send_msp_tid;
    pthread_t received_msp_tid;

    pthread_create(&send_msp_tid
            , NULL
            , send_msp_thread
            , NULL);
    pthread_detach(send_msp_tid);

    pthread_create(&received_msp_tid
            , NULL
            , received_msp_thread
            , NULL);
    pthread_detach(received_msp_tid);
}

void msp_arm()
{
    msp_arm_request = 1;
}

void msp_disarm()
{
    msp_disarm_request = 1;
}

void msp_acc_calib()
{
    msp_acc_calib_request = 1;
}

void msp_mag_calib()
{
    msp_mag_calib_request = 1;
}

void msp_eeprom_write()
{
    msp_eeprom_write_request = 1;
}

void msp_get_att(att_t* att_info)
{
    memcpy(att_info, &att, sizeof(att_t));
}

void msp_get_alt(alt_t* alt_info)
{
    memcpy(alt_info, &alt, sizeof(alt_t));
}

void msp_get_imu(imu_t* imu_info)
{
    memcpy(imu_info, &imu, sizeof(imu_t));
}

void msp_get_status(msp_status_t* msp_status_info)
{
    memcpy(msp_status_info, &msp_status, sizeof(msp_status));
}

void msp_get_debug(int16_t* debug_info)
{
    memcpy(debug_info, debug, sizeof(debug));
}

void msp_left()
{
    roll_input = -50;
}

void msp_right()
{
    roll_input = 50;
}

void msp_forward()
{
    pitch_input = 50;
}

void msp_backward()
{
    pitch_input = -50;
}

void msp_turn_left()
{
    yaw_input = -100;
}

void msp_turn_right()
{
    yaw_input = 100;
}

void msp_attitude_input_default()
{
    roll_input = 0;
    pitch_input = 0;
    yaw_input = 0;
}

void msp_throttle(int16_t throttle)
{
    throttle_input = throttle;
}

void msp_set_alt_mod()
{
    althold_switch_input = 800;
}

void msp_reset_alt_mod()
{
    althold_switch_input = 0;
}
