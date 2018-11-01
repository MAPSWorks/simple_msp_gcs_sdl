#include "video_stream_part.h"
#include "video_log.h"
#include "../msp_protocol/msp_protocol.h"

#include "../rpi-udp-stream-client/udp_setup/udp_setup.h"
#include "../rpi-udp-stream-client/common_util/common_util.h"
#include "../rpi-udp-stream-client/ffmpeg_setup/ffmpeg_setup.h"
#include "../rpi-udp-stream-client/computer_vision/imshow_queue.h"
#include "../rpi-udp-stream-client/computer_vision/color_object_recognition.h"
#include "../rpi-udp-stream-client/computer_vision/shape_object_recognition.h"
#include "../rpi-udp-stream-client/computer_vision/get_optical_flow.h"
#include "../rpi-udp-stream-client/computer_vision/pos_compensation.h"

#include "shape_detection_mode.h"

#include "opencv2/opencv.hpp"
#include <string>
using namespace cv;

static pthread_t receive_id;
static pthread_t keep_alive_id;

static uint8_t video_quit = 1;

static opt_flow_t flow_glov;
static detected_position_t maker_glov;

void set_video_quit()
{
    video_quit = 1;
}

void reset_video_quit()
{
    video_quit = 0;
}

uint8_t is_video_quit()
{
    return video_quit;
}

void avframe_mat_conversion(const AVFrame * frame, Mat& image)
{
    int width = frame->width;
    int height = frame->height;

    // Allocate the opencv mat and store its stride in a 1-element array
    if (image.rows != height 
            || image.cols != width 
            || image.type() != CV_8UC3)
    {
        image = Mat(height, width, CV_8UC3);
    }

    int cvLinesizes[1];
    cvLinesizes[0] = image.step1();

    // Convert the colour format and write directly to the opencv matrix
    SwsContext* conversion = sws_getContext(width
            , height
            , (AVPixelFormat) frame->format
            , width
            , height
            , PIX_FMT_BGR24
            , SWS_FAST_BILINEAR
            , NULL, NULL, NULL);

    sws_scale(conversion
            , frame->data
            , frame->linesize
            , 0
            , height
            , &image.data
            , cvLinesizes);

    sws_freeContext(conversion);
}

static void* receive_video_udp(void* arg)
{
    uint32_t frame_len;
    uint8_t frame_buf[FRAME_BUFSIZE];

#ifdef SAVE_VIDEO
    int fd = open (FILENAME, O_WRONLY | O_CREAT | O_TRUNC | O_APPEND, 0666);
    if (fd == -1)
    {
        DEBUG_ERR("error: open\n");
        exit(1);
    }
#endif

    ffmpeg_decode_init();

    Mat converted_image;
    imshow_init();
    color_object_init();
    shape_init();

    while(!(is_quit() || video_quit))
    {
        if(udp_recevie_stream(frame_buf, &frame_len))
            DEBUG_MSG("frame received size : %d\n", frame_len);

        AVFrame picture;
        if(ffmpeg_decode_start(frame_buf, frame_len, &picture))
        {
            DEBUG_MSG("YUV frame successfully decoded\n");
            
            avframe_mat_conversion(&picture, converted_image);

            //get drone's information
            drone_info_t drone_info;
            msp_get_info(&drone_info);

            //color object detection
            color_object_t red_obj;
            color_object_t green_obj;
            color_object_t blue_obj;
            find_red_object(converted_image, &red_obj);
            find_green_object(converted_image, &green_obj);
            find_blue_object(converted_image, &blue_obj);

            imshow_request("red", red_obj.thresholded_image);
            imshow_request("blue", blue_obj.thresholded_image);
            imshow_request("green", green_obj.thresholded_image);

            //shape detection
            shape_object_t red_obj_shape;
            shape_object_t green_obj_shape;
            shape_object_t blue_obj_shape;
            check_shape(red_obj.thresholded_image, &red_obj_shape);
            check_shape(green_obj.thresholded_image, &green_obj_shape);
            check_shape(blue_obj.thresholded_image, &blue_obj_shape);

            imshow_request("red_edge", red_obj_shape.thresholded_image);
            imshow_request("blue_edge", blue_obj_shape.thresholded_image);
            imshow_request("green_edge", green_obj_shape.thresholded_image);

            //pos_compensation
            Point2i actual_position;
            actual_position = find_position_in_image(converted_image, drone_info.angle[0], drone_info.angle[1], drone_info.height);
            
            //find position of drone in world coordinate
            detected_position_t detected_position;
            if(get_postion_from_marker(actual_position,
                                        &red_obj,
                                        &blue_obj,
                                        &green_obj,
                                        &red_obj_shape,
                                        &blue_obj_shape,
                                        &green_obj_shape,
                                        &detected_position))
            {
                memcpy(&maker_glov, &detected_position, sizeof(detected_position));
                //draw object shape infomation
                Mat mask;
                converted_image.copyTo(mask);
                for(int i = 0; i < red_obj_shape.detected_num; i++)
                {
                    draw_label(mask, red_obj_shape.type[i], red_obj_shape.position[i]);
                }

                for(int i = 0; i < green_obj_shape.detected_num; i++)
                {
                    draw_label(mask, green_obj_shape.type[i], green_obj_shape.position[i]);
                }

                for(int i = 0; i < blue_obj_shape.detected_num; i++)
                {
                    draw_label(mask, blue_obj_shape.type[i], blue_obj_shape.position[i]);
                }
                imshow_request("shape_test", mask);

                // draw detected position info
                Mat pos_mask;
                converted_image.copyTo(pos_mask);

                circle(pos_mask, actual_position, 5, Scalar(125,125,125), CV_FILLED, 1, 0);
                for(int i = 0; i < red_obj_shape.detected_num; i++)
                {
                    circle(pos_mask, red_obj_shape.position[i], 5, Scalar(0,0,255), CV_FILLED, 1, 0);
                }
                for(int i = 0; i < blue_obj_shape.detected_num; i++)
                {
                    circle(pos_mask, blue_obj_shape.position[i], 5, Scalar(255,125,0), CV_FILLED, 1, 0);
                }
                for(int i = 0; i < green_obj_shape.detected_num; i++)
                {
                    circle(pos_mask, green_obj_shape.position[i], 5, Scalar(0,255,0), CV_FILLED, 1, 0);
                }
                string pos_string = std::to_string(detected_position.x) + ',' + std::to_string(detected_position.y);
                string pixel_diff_string = std::to_string(detected_position.x_diff) + ',' + std::to_string(detected_position.y_diff);
                cv::putText(pos_mask, "World coordination " + pos_string, Point(20,20), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(0, 125, 255), 2, 8);	
                cv::putText(pos_mask, "Pixel difference " + pixel_diff_string, Point(20,40), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(0, 125, 255), 2, 8);
                line(pos_mask
                    , actual_position
                    , Point(actual_position.x + detected_position.x_diff, actual_position.y + detected_position.y_diff)
                    , Scalar(255, 255, 0));
                imshow_request("position_info", pos_mask);
            }

            //optical flow
            opt_flow_t flow_info;
            get_optical_flow(converted_image, &flow_info);
            memcpy(&flow_glov, &flow_info, sizeof(flow_info));

            if(!flow_info.bad_condition)
            {
                imshow_request("masked_img", flow_info.masked_img);
            }

            save_this_frame(converted_image);
            imshow_request("convert", converted_image);
        }

#ifdef SAVE_VIDEO
        if (write(fd
                    , frame_buf
                    , frame_len) == -1)
        {
            DEBUG_ERR("error: write\n");
            exit(1);
        }
#endif
    }

#ifdef SAVE_VIDEO
    close(fd);
#endif

    imshow_close();

    ffmpeg_decode_close();

    DEBUG_MSG("video stream thread ended\n");
    pthread_exit((void *) 0);
}

static void* keep_alive(void* arg)
{
    while(!(is_quit() || video_quit))
    {
        udp_send_command("SET_TIMEOUT");
        sleep(1);
    }

    DEBUG_MSG("video stream keep_alive ended\n");
    pthread_exit((void *) 0);
}

void video_init(char* ip)
{
    udp_set_server_ip(ip);
    udp_client_command_setup();
    udp_client_stream_setup();
}

void video_deinit()
{
    DEBUG_MSG("close and exit client\n");
    udp_client_command_close();
    udp_client_stream_close();
}

void video_start()
{
    reset_video_quit();
    
    pthread_create(&receive_id
            , NULL
            , receive_video_udp
            , (void*)NULL);
    DEBUG_MSG("receive_video_udp thread created\n");
    pthread_detach(receive_id);

    pthread_create(&keep_alive_id
            , NULL
            , keep_alive
            , (void*)NULL);
    DEBUG_MSG("keep_alive thread created\n");
    pthread_detach(keep_alive_id);

    udp_send_command("VIDEO_REQUEST");
}

void video_stop()
{
    set_video_quit();
}

void get_opt_flow_data(opt_flow_t* flow)
{
    memcpy(flow, &flow_glov, sizeof(flow_glov));
}

void get_maker_track_data(detected_position_t* marker)
{
    memcpy(marker, &maker_glov, sizeof(maker_glov));   
}