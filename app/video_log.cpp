#include "video_log.h"
#include "../msp_protocol/msp_protocol.h"

#include <stdint.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

#define VIDEO_WIDTH 320
#define VIDEO_HEIGHT 240
#define FPS 25

#define VIDEO_FILE_NAME "log_video_file.avi"
#define SENSOR_TEXT_NAME "log_video_sensor_file.txt"

static ofstream log_video_sensor_file;
static VideoWriter log_video_file;

static bool is_open = false;

void video_log_init()
{
    if(!is_open)
    {
        is_open = true;
        log_video_file.open(VIDEO_FILE_NAME, CV_FOURCC('M', 'P', '4', '2'), FPS, Size(VIDEO_WIDTH, VIDEO_HEIGHT), true);
        log_video_sensor_file.open(SENSOR_TEXT_NAME, ios::out|ios::trunc);
    }
}

void video_log_deinit()
{
    if(is_open)
    {
        is_open = false;
        log_video_file.release();
        log_video_sensor_file.close();
    }
}

void save_this_frame(cv::Mat frame)
{
    if(is_open)
    {
        drone_info_t drone_info;
        msp_get_info(&drone_info);

        log_video_file << frame;
        
        log_video_sensor_file << drone_info.angle[0] 
                         << '\t' << drone_info.angle[1] 
                         << '\t' << drone_info.height
                         << '\t' << endl;
    }
}
