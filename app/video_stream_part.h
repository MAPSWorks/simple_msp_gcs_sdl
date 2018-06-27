#ifndef VIDEO_STREAM_PART_H
#define VIDEO_STREAM_PART_H

#ifdef SAVE_VIDEO
#define FILENAME "video.h264"
#endif

#include <stdint.h>

#include "../rpi-udp-stream-client/computer_vision/color_object_recognition.h"
#include "../rpi-udp-stream-client/computer_vision/get_optical_flow.h"

void set_video_quit();
void reset_video_quit();
uint8_t is_video_quit();

void video_init(char* ip);
void video_deinit();
void video_start();
void video_stop();

void get_opt_flow_data(opt_flow_t* flow);

#endif
