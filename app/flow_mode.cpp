#include "flow_mode.h"

#include "../rpi-udp-stream-client/common_util/common_util.h" //use utils from rpi-udp-stream-client submodule

static float flow[2];
static int16_t gyro[2];
static int32_t altitude; //in cm
static int16_t flow_compensated[2];

void flow_mode_set_flow(float flow_x, float flow_y)
{
    //check direction of flow
    flow[0] = flow_x;
    flow[1] = -flow_y;
}

void flow_mode_set_gyro(int16_t gyro_roll, int16_t gyro_pitch)
{
    //check direction of gyro sensor
    gyro[0] = gyro_roll;
    gyro[1] = gyro_pitch;
}

void flow_mode_set_altitude(int32_t height)
{
    altitude = height;
}

void do_flow_mode()
{
    //to remove rotation effect of optical flow, combine with gyro sensor's value
    /*
    const float flow_gain = 23.33f; //To match the gyro sensor value to the optical flow value
    float altitude_float = altitude / 100.0f; // 1 m based altitude compensate
    float output_adjust_gain = 10.0f;
    
    for(int i = 0; i < 2; i++)
    {
        flow_compensated[i] = static_cast<int16_t>((flow_gain*(altitude_float*flow[i]) - gyro[i]) / output_adjust_gain);
    }

    DEBUG_MSG("flow_x : %d, flow_y : %d\n", flow_compensated[0], flow_compensated[1]);
    */

    const int gyro_th = 150;
    float altitude_float = altitude / 100.0f; // 1 m based altitude compensate

    static int16_t pre_flow[2] = {0, 0};
    int16_t current_flow[2];

    const float tau = 0.05;
    const float ts = 0.02;

    const float output_gain = 10.0f;

    for(int i = 0; i < 2; i++)
    {
        if((abs(gyro[0]) > gyro_th) || (abs(gyro[1]) > gyro_th))
            current_flow[i] = 0;
        else
            current_flow[i] = output_gain*altitude_float*flow[i];

        flow_compensated[i] = (tau*pre_flow[i] + ts*current_flow[i])/(tau +ts);

        pre_flow[i] = flow_compensated[i];
    }
}

void get_flow_output(int16_t* x, int16_t* y)
{
    *x = flow_compensated[0];
    *y = flow_compensated[1];
}