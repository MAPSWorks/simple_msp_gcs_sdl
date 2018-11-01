#include "shape_detection_mode.h"
#include "../rpi-udp-stream-client/common_util/common_util.h"
#include "opencv2/opencv.hpp"
#include <vector>
using namespace std;
using namespace cv;

static bool filter_noise(detected_position_t* input, detected_position_t* output)
{
    bool retval = false;

    static int sample_count = 0;
    const int max_sample = 3;

    static vector<int> x(max_sample);
    static vector<int> y(max_sample);
    static vector<int> x_diff(max_sample);
    static vector<int> y_diff(max_sample);

    x.push_back(input->x);
    y.push_back(input->y);
    x_diff.push_back(input->x_diff);
    y_diff.push_back(input->y_diff);
    sample_count++;

    if(sample_count == 3)
    {
        sample_count = 0;

        sort(x.begin(), x.end());
        sort(y.begin(), y.end());
        sort(x_diff.begin(), x_diff.end());
        sort(y_diff.begin(), y_diff.end());

        output->x = x[1];
        output->y = y[1];
        output->x_diff = x_diff[1];
        output->y_diff = y_diff[1];
        output->detected = 1;

        x.clear();
        y.clear();
        x_diff.clear();
        y_diff.clear();

        retval = true;
    }

    return retval;
}

bool get_postion_from_marker(Point2i actual_position,
                             color_object_t* red_obj, 
                             color_object_t* blue_obj,
                             color_object_t* green_obj, 
                             shape_object_t* red_shape,
                             shape_object_t* blue_shape,
                             shape_object_t* green_shape, 
                             detected_position_t* detected_position)
{
    bool retval = false;
    
    //default info of detected position structure
    detected_position->detected = false;
    detected_position->x = 0;
    detected_position->y = 0;
    detected_position->x_diff = 0;
    detected_position->y_diff = 0;

    //something detected?
    if(red_obj->is_recognized || green_obj->is_recognized || blue_obj->is_recognized)
    {
        for(int i = 0; i < red_shape->detected_num; i++)
        {
            switch(red_shape->type[i])
            {
                case SHAPE_RECT:
                detected_position->detected = 1;
                detected_position->x = 0;
                detected_position->y = 0;
                detected_position->x_diff = red_shape->position[i].x - actual_position.x;
                detected_position->y_diff = red_shape->position[i].y - actual_position.y;
                break;

                case SHAPE_TRI:
                detected_position->detected = 1;
                detected_position->x = 0;
                detected_position->y = 1;
                detected_position->x_diff = red_shape->position[i].x - actual_position.x;
                detected_position->y_diff = red_shape->position[i].y - actual_position.y;
                break;

                case SHAPE_CIRCLE:
                detected_position->detected = 1;
                detected_position->x = 0;
                detected_position->y = 2;
                detected_position->x_diff = red_shape->position[i].x - actual_position.x;
                detected_position->y_diff = red_shape->position[i].y - actual_position.y;
                break;

                default :
                detected_position->detected = 0;
                detected_position->x = 0;
                detected_position->y = 0;
                detected_position->x_diff = 0;
                detected_position->y_diff = 0;
                break;
            }
        }

        for(int i = 0; i < blue_shape->detected_num; i++)
        {
            switch(blue_shape->type[i])
            {
                case SHAPE_RECT:
                detected_position->detected = 1;
                detected_position->x = 1;
                detected_position->y = 0;
                detected_position->x_diff = blue_shape->position[i].x - actual_position.x;
                detected_position->y_diff = blue_shape->position[i].y - actual_position.y;
                break;

                case SHAPE_TRI:
                detected_position->detected = 1;
                detected_position->x = 1;
                detected_position->y = 1;
                detected_position->x_diff = blue_shape->position[i].x - actual_position.x;
                detected_position->y_diff = blue_shape->position[i].y - actual_position.y;
                break;

                case SHAPE_CIRCLE:
                detected_position->detected = 1;
                detected_position->x = 1;
                detected_position->y = 2;
                detected_position->x_diff = blue_shape->position[i].x - actual_position.x;
                detected_position->y_diff = blue_shape->position[i].y - actual_position.y;
                break;

                default :
                detected_position->detected = 0;
                detected_position->x = 0;
                detected_position->y = 0;
                detected_position->x_diff = 0;
                detected_position->y_diff = 0;
                break;
            }
        }

        for(int i = 0; i < green_shape->detected_num; i++)
        {
            switch(green_shape->type[i])
            {
                case SHAPE_RECT:
                detected_position->detected = 1;
                detected_position->x = 2;
                detected_position->y = 0;
                detected_position->x_diff = green_shape->position[i].x - actual_position.x;
                detected_position->y_diff = green_shape->position[i].y - actual_position.y;
                break;

                case SHAPE_TRI:
                detected_position->detected = 1;
                detected_position->x = 2;
                detected_position->y = 1;
                detected_position->x_diff = green_shape->position[i].x - actual_position.x;
                detected_position->y_diff = green_shape->position[i].y - actual_position.y;
                break;

                case SHAPE_CIRCLE:
                detected_position->detected = 1;
                detected_position->x = 2;
                detected_position->y = 2;
                detected_position->x_diff = green_shape->position[i].x - actual_position.x;
                detected_position->y_diff = green_shape->position[i].y - actual_position.y;
                break;

                default :
                detected_position->detected = 0;
                detected_position->x = 0;
                detected_position->y = 0;
                detected_position->x_diff = 0;
                detected_position->y_diff = 0;
                break;
            }
        }
        
        if(detected_position->detected)
        {
            if(filter_noise(detected_position, detected_position))
            {
                retval = true;
            }
        }
    }

    return retval;
}