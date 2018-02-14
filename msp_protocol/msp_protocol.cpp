#include "msp_protocol.h"

#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include "../serial_setup/serial_setup.h"

static att_t att;
static alt_t alt;

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

static void msp_write_cmd(uint8_t cmd)
{
    serial_write('$');
    serial_write('M');
    serial_write('<');
    serial_write(0);
    serial_write(cmd);
    serial_write(cmd);
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

static void* send_msp_thread(void* arg)
{
    static uint8_t cmd_state = 0;

    while(1)
    {
        switch(cmd_state)
        {
            case 0:
                msp_write_cmd(MSP_ATTITUDE);
                cmd_state++;
                break;
            case 1:
                msp_write_cmd(MSP_ALTITUDE);
                cmd_state++;
                break;
            default :
                cmd_state = 0;
                break;
        }

        usleep(1000);
    }
}

static void* received_msp_thread(void* arg)
{
    uint8_t received_cmd;
    uint8_t received_size;
    uint8_t received_buf[MAXBUF_SIZE];

    while(1)
    {
        if(msp_parse_cmd(&received_cmd, &received_size, received_buf))
        {
            switch(received_cmd)
            {
                case MSP_ATTITUDE:
                    memcpy((uint8_t*)&att, received_buf, received_size);
                    break;
                case MSP_ALTITUDE:
                    memcpy((uint8_t*)&alt, received_buf, received_size);
                    break;
                default :
                    break;
            }
        }
        usleep(100);
    }
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
    msp_write_cmd(MSP_ARM);
}

void msp_disarm()
{
    msp_write_cmd(MSP_DISARM);
}

void msp_get_att(att_t* att_info)
{
    memcpy(att_info, &att, sizeof(att_t));
}

void msp_get_alt(alt_t* alt_info)
{
    memcpy(alt_info, &alt, sizeof(alt_t));
}
