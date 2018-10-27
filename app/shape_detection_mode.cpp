#include "shape_detection_mode.h"

#include "opencv2/opencv.hpp"
using namespace cv;

void get_postion_from_marker(Point2i actual_position,
                             color_object_t* red_obj, 
                             color_object_t* blue_obj,
                             color_object_t* green_obj, 
                             shape_object_t* red_shape,
                             shape_object_t* blue_shape,
                             shape_object_t* green_shape, 
                             detected_position_t* detected_position)
{
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
    }
}