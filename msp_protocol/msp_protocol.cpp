#include "msp_protocol.h"

#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include "../serial_setup/serial_setup.h"
#include "../rpi-udp-stream-client/common_util/common_util.h"

#include <iostream>
#include <fstream>

static uint8_t request_drone_info = 0;
static drone_info_t drone_info_tmp = {0, };

static int16_t roll_input = 0;
static int16_t pitch_input = 0;
static int16_t yaw_input = 0;
static uint8_t throttle_input = 0;
static int16_t althold_switch_input = 0;

static uint8_t msp_arm_request = 0;
static uint8_t msp_disarm_request = 0;
static uint8_t msp_acc_calib_request = 0;
static uint8_t msp_mag_calib_request = 0;
static uint8_t msp_eeprom_write_request = 0;

static int16_t flow_output[2];

enum
{
    TRIM_UP = 0,
    TRIM_DOWN,
    TRIM_LEFT,
    TRIM_RIGHT,
};

static uint8_t msp_trim_request = 0;
static uint8_t msp_trim_state = 0;

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
    RC_MAX = 250,
    RC_MIN = 0,
    RC_MID = 125,

    rcRoll = 0,
    rcPitch,
    rcYaw,
    rcThrottle,
    rcAux1,
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

static uint8_t is_test_mode = 0;
static uint32_t test_send_interval = 0;
static uint16_t test_count = 0;
static uint8_t test_sending_size = 0;
static uint8_t test_receiving_size = 0;
static uint8_t test_received_size = 0;
static uint16_t test_cycle_time = 0;
static uint16_t test_cycle_time_max = 0;
static uint16_t sending_count = 0;
static uint16_t receiving_count = 0;

static uint16_t received_test_counts[10000];
static uint16_t received_test_counts_idx = 0;

static void msp_test_send(uint8_t test_send_size, uint8_t test_receive_size, uint32_t send_interval, uint16_t test_count)
{
    if(test_send_size < 3)
        test_send_size = 3;

    if(test_receive_size < 4)
        test_receive_size = 4;

    uint8_t checksum = 0;
    checksum = MSP_TEST^test_send_size;

    uint8_t test_count_lb, test_count_hb;
    test_count_lb = (test_count & 0xff);
    test_count_hb = ((test_count >> 8) & 0xff);

    serial_write('$');
    serial_write('M');
    serial_write('<');
    serial_write(test_send_size);
    serial_write(MSP_TEST);

    serial_write(test_receive_size);
    checksum ^= test_receive_size;
    
    serial_write(test_count_lb);
    checksum ^= test_count_lb;
    
    serial_write(test_count_hb);
    checksum ^= test_count_hb;
    
    for(uint8_t i = 0; i < (test_send_size - 3); i++)
    {
        serial_write((uint8_t)0);
        checksum ^= (uint8_t)0;
    }
    serial_write(checksum);
    sending_count++;
    usleep(send_interval);
}

static void msp_test_receive(uint8_t received_size, uint8_t* received_buf)
{
    uint16_t* buf_16 = (uint16_t*)received_buf;

    test_received_size = received_size;
    test_cycle_time = buf_16[0];

    received_test_counts[received_test_counts_idx] = buf_16[1];
    received_test_counts_idx++;
    
    if(test_cycle_time > test_cycle_time_max)
        test_cycle_time_max = test_cycle_time;
    receiving_count++;
}

void msp_test_enable(uint8_t enable)
{
    if(enable == 1)
        is_test_mode = 1;
    else if(enable == 0)
        is_test_mode = 0;
}

void msp_test_set(uint8_t send_size, uint8_t receive_size, uint32_t send_interval, uint16_t send_count)
{
    sending_count = 0;
    receiving_count = 0;
    test_cycle_time = 0;
    test_cycle_time_max = 0;
    test_received_size = 0;

    test_send_interval = send_interval;
    test_count = send_count;
    test_sending_size = send_size;
    test_receiving_size = receive_size;
}

void msp_test_get_info(uint16_t* send_count, uint16_t* receive_count, uint16_t* drone_cycle_time, uint16_t* drone_cycle_time_max, uint8_t* received_size)
{
    *send_count = sending_count;
    *receive_count = receiving_count;
    *drone_cycle_time = test_cycle_time;
    *drone_cycle_time_max = test_cycle_time_max;
    *received_size = test_received_size;
}

void msp_test_save()
{
    std::ofstream msp_test;

    msp_test.open("msp_test_result", std::ios::out|std::ios::trunc);

    msp_test << "Tested count : " << sending_count << std::endl;
    msp_test << "Received count : " << receiving_count << std::endl;

    for(int i = 0; i < received_test_counts_idx; i++)
    {
        msp_test << received_test_counts[i] << std::endl;
    }
    
    msp_test.close();
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

    while(serial_available())
    {
        c = serial_read();
        switch(state)
        {
            case WAIT_$ :
                state = (c == '$') ? WAIT_M : WAIT_$;
                break;
            case WAIT_M :
                state = (c == 'M') ? WAIT_ARROW : WAIT_$;
                if(c != 'M')
                    DEBUG_MSG("error in msp receive\n");
                break;
            case WAIT_ARROW :
                state = (c == '>') ? WAIT_BUFSIZE : WAIT_$;
                if(c != '>')
                    DEBUG_MSG("error in msp receive\n");
                break;
            case WAIT_BUFSIZE :
                if(c > MAXBUF_SIZE)
                {
                    state = WAIT_$; //buffer size error
                    DEBUG_MSG("error in msp receive\n");
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
                        DEBUG_MSG("error in msp receive\n");
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
    uint8_t rc[5];
    rc[rcRoll]      = RC_MID + roll_input + flow_output[0];
    rc[rcPitch]     = RC_MID + pitch_input + flow_output[1];
    rc[rcYaw]       = RC_MID + yaw_input;
    rc[rcThrottle]  = RC_MIN + throttle_input;
    rc[rcAux1]      = RC_MIN + althold_switch_input;

    msp_write_buf(MSP_SET_TINY_RC, rc, 5);
}

static void* send_msp_thread(void* arg)
{
    static uint8_t cmd_state = 0;

    while(!is_quit())
    {
        if(!is_test_mode)
        {
            switch(cmd_state)
            {
                case 0:
                    if(request_drone_info)
                    {
                        msp_write_cmd(MSP_SENSORS);
                        request_drone_info = 0;
                    }
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
                    if(msp_trim_request)
                    {
                        switch(msp_trim_state)
                        {
                            case TRIM_UP:
                                msp_write_cmd(MSP_TRIM_UP);
                                break;
                            case TRIM_DOWN:
                                msp_write_cmd(MSP_TRIM_DOWN);
                                break;
                            case TRIM_LEFT:
                                msp_write_cmd(MSP_TRIM_LEFT);
                                break;
                            case TRIM_RIGHT:
                                msp_write_cmd(MSP_TRIM_RIGHT);
                                break;
                            default :
                                break;
                        }
                        msp_trim_request = 0;
                    }
                    cmd_state++;
                    break;
                case 1:
                    msp_send_rc();
                    cmd_state++;
                    break;
                default:
                    cmd_state = 0;
                    break;
            }

            usleep(10000);
        }
        else //testing mode
        {
            if(test_count != 0)
            {
                msp_test_send(test_sending_size, test_receiving_size, test_send_interval, test_count);
                test_count--;
            }
        }
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
                case MSP_SENSORS:
                    memcpy((uint8_t*)&drone_info_tmp, received_buf, received_size);
                    break;
                case MSP_TEST:
                    msp_test_receive(received_size, received_buf);
                    break;
                default :
                    break;
            }
        }
        usleep(50);
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

void msp_left()
{
    roll_input = -25;
}

void msp_right()
{
    roll_input = 25;
}

void msp_forward()
{
    pitch_input = 25;
}

void msp_backward()
{
    pitch_input = -25;
}

void msp_turn_left()
{
    yaw_input = -25;
}

void msp_turn_right()
{
    yaw_input = 25;
}

void msp_attitude_input_default()
{
    roll_input = 0;
    pitch_input = 0;
    yaw_input = 0;
}

void msp_throttle(uint8_t throttle)
{
    throttle_input = throttle;
}

void msp_set_alt_mod()
{
    althold_switch_input = 200;
}

void msp_reset_alt_mod()
{
    althold_switch_input = 0;
}

void msp_request_info()
{
    request_drone_info = 1;
}

void msp_get_info(drone_info_t* drone_info)
{
    memcpy(drone_info, &drone_info_tmp, sizeof(drone_info_tmp));
}

void msp_set_trim_up()
{
    msp_trim_request = 1;
    msp_trim_state = TRIM_UP;
}

void msp_set_trim_down()
{
    msp_trim_request = 1;
    msp_trim_state = TRIM_DOWN;
}

void msp_set_trim_left()
{
    msp_trim_request = 1;
    msp_trim_state = TRIM_LEFT;
}

void msp_set_trim_right()
{
    msp_trim_request = 1;
    msp_trim_state = TRIM_RIGHT;
}

void msp_set_flow_output(int16_t x, int16_t y)
{
    if(x > 25)
        flow_output[0] = 25;
    else if(x < -25)
        flow_output[0] = -25;
    else
        flow_output[0] = x;

    if(y > 25)
        flow_output[1] = 25;
    else if(y < -25)
        flow_output[1] = -25;
    else
        flow_output[1] = y;
}
