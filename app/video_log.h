#ifndef VIDEO_LOG_H
#define VIDEO_LOG_H

#include "opencv2/opencv.hpp"

void video_log_init();
void video_log_deinit();
void save_this_frame(cv::Mat frame);

#endif
