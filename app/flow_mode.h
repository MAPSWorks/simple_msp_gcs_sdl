#ifndef FLOW_MODE_H
#define FLOW_MODE_H

#include "video_stream_part.h"

void flow_mode_set_flow(float flow_x, float flow_y);
void flow_mode_set_gyro(int16_t gyro_roll, int16_t gyro_pitch);
void flow_mode_set_altitude(int32_t height);
void get_flow_output(int16_t* x, int16_t* y);
void do_flow_mode();

#endif