#ifndef SHAPE_DETECTION_MODE_H
#define SHAPE_DETECTION_MODE_H

#include "../rpi-udp-stream-client/computer_vision/shape_object_recognition.h"
#include "../rpi-udp-stream-client/computer_vision/color_object_recognition.h"

typedef struct detected_position_t
{
    //expected drone's postion in world coordinate
    bool detected;
    int x;
    int y;

    //difference of drone's position and marker's position
    int x_diff;
    int y_diff;
} detected_position_t;

bool get_postion_from_marker(cv::Point2i actual_position, 
                             color_object_t* red_obj, 
                             color_object_t* blue_obj,
                             color_object_t* green_obj, 
                             shape_object_t* red_shape,
                             shape_object_t* blue_shape,
                             shape_object_t* green_shape, 
                             detected_position_t* detected_position);

#endif