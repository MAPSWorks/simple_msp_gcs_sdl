#include "./serial_setup/serial_setup.h"
#include "./msp_protocol/msp_protocol.h"
#include "./gui/gui.h"

#include <stdint.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <stdio.h>
#include <string.h>

using namespace std;
using namespace nanogui;

#include <SDL2/SDL.h>

#include "./rpi-udp-stream-client/udp_setup/udp_setup.h"
#include "./rpi-udp-stream-client/common_util/common_util.h"
#include "./rpi-udp-stream-client/ffmpeg_setup/ffmpeg_setup.h"
#include "./rpi-udp-stream-client/computer_vision/imshow_queue.h"
#include "./rpi-udp-stream-client/computer_vision/color_object_recognition.h"

#include "opencv2/opencv.hpp"
using namespace cv;

pthread_t receive_id;
int receive_status;
pthread_t keep_alive_id;
int keep_alive_status;

#define MAX_PLOT_SIZE 100

#ifdef SAVE_VIDEO
#define FILENAME "video.h264"
#endif

static void sig_int(int arg)
{
    DEBUG_MSG("request server close and exit\n");
    udp_send_command("QUIT_SERVER");

    set_quit();
    DEBUG_MSG("close and exit client\n");
    udp_client_command_close();
    udp_client_stream_close();
    
    DEBUG_MSG("wait until thread join\n");
    pthread_join(receive_id, (void **)&receive_status);
    pthread_join(keep_alive_id, (void **)&keep_alive_status);
 
    exit(0);
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

    while(!is_quit())
    {
        if(udp_recevie_stream(frame_buf, &frame_len))
            DEBUG_MSG("frame received size : %d\n", frame_len);
        
        AVFrame picture;
        if(ffmpeg_decode_start(frame_buf, frame_len, &picture))
        {
            DEBUG_MSG("YUV frame successfully decoded\n");
            //DEBUG_MSG("picture linesize 0 : %d\n", picture.linesize[0]);
            //DEBUG_MSG("picture linesize 1 : %d\n", picture.linesize[1]);
            //DEBUG_MSG("picture linesize 2 : %d\n", picture.linesize[2]);

            avframe_mat_conversion(&picture, converted_image);

            color_object_t red_obj;
            color_object_t green_obj;
            color_object_t blue_obj;
            find_red_object(converted_image, &red_obj);
            find_green_object(converted_image, &green_obj);
            find_blue_object(converted_image, &blue_obj);

            if(red_obj.is_recognized)
                imshow_request("red_obj", red_obj.thresholded_image);
            if(green_obj.is_recognized)
                imshow_request("green_obj", green_obj.thresholded_image);
            if(blue_obj.is_recognized)
                imshow_request("blue_obj", blue_obj.thresholded_image);
            
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
    
    pthread_exit((void *) 0);
}

static void* keep_alive(void* arg)
{
    while(!is_quit())
    {
        udp_send_command("SET_TIMEOUT");
        sleep(1);
    }

    pthread_exit((void *) 0);
}

int main(int argc, char* argv[])
{
    if(argc != 2) 
    {
       DEBUG_ERR("Usage: %s IP_address\n", argv[0]);
       exit(1);
    }

    //signal interrupt
    signal(SIGINT,  sig_int);
    signal(SIGTERM, sig_int);
    signal(SIGQUIT, sig_int);
    
    udp_set_server_ip(argv[1]);
    udp_client_command_setup();
    udp_client_stream_setup();

    serial_init();
    msp_init();

    att_t att = {9999, };
    alt_t alt = {9999, };
    imu_t imu = {9999, };
    int16_t debug[4] = {9999, };

    gui_init();

    FormHelper* gui = gui_get_pointer();
    
    nanogui::ref<Window> rwindow = gui->addWindow(Eigen::Vector2i(10, 10), "Basic information");
    gui->addGroup("attitude info");
    gui->addVariable("angle0", att.angle[0]);
    gui->addVariable("angle1", att.angle[1]);
    gui->addVariable("heading", att.heading);

    gui->addGroup("altitude info");
    gui->addVariable("EstAlt", alt.EstAlt);
    gui->addVariable("vario", alt.vario);

    gui->addGroup("Debug Data");
    gui->addVariable("Debug[0]", debug[0]);
    gui->addVariable("Debug[1]", debug[1]);
    gui->addVariable("Debug[2]", debug[2]);
    gui->addVariable("Debug[3]", debug[3]);

    nanogui::ref<Window> rwindow2 = gui->addWindow(Eigen::Vector2i(210, 10), "Raw sensor data");
    gui->addGroup("Raw sensor data");
    gui->addVariable("accSmooth 0", imu.accSmooth[0]);
    gui->addVariable("accSmooth 1", imu.accSmooth[1]);
    gui->addVariable("accSmooth 2", imu.accSmooth[2]);

    gui->addVariable("gyroData 0", imu.gyroData[0]);
    gui->addVariable("gyroData 1", imu.gyroData[1]);
    gui->addVariable("gyroData 2", imu.gyroData[2]);

    gui->addVariable("magADC 0", imu.magADC[0]);
    gui->addVariable("magADC 1", imu.magADC[1]);
    gui->addVariable("magADC 2", imu.magADC[2]);
    
    gui->addVariable("gyroADC 0", imu.gyroADC[0]);
    gui->addVariable("gyroADC 1", imu.gyroADC[1]);
    gui->addVariable("gyroADC 2", imu.gyroADC[2]);
    
    gui->addVariable("accADC 0", imu.accADC[0]);
    gui->addVariable("accADC 1", imu.accADC[1]);
    gui->addVariable("accADC 2", imu.accADC[2]);

    nanogui::ref<Window> rwindow3 = gui->addWindow(Eigen::Vector2i(10, 385), "Arming");
    gui->addGroup("Arming");
    gui->addButton("Arm", []()
            {
                std::cout << "Arm" << std::endl;
                msp_arm();
            });
    gui->addButton("Disarm", []()
            {
                std::cout << "Disarm" << std::endl;
                msp_disarm();
            });
    gui->addGroup("Calibration");
    gui->addButton("Acc calibration", []()
            {
                std::cout << "Acc calibration" << std::endl;
                msp_acc_calib();
            });
    gui->addButton("Mag calibration", []()
            {
                std::cout << "Mag calibration" << std::endl;
                msp_mag_calib();
            });
    gui->addGroup("Save setting");
    gui->addButton("EEPROM write", []()
            {
                std::cout << "EEPROM write" << std::endl;
                msp_eeprom_write();
            });

    nanogui::ref<Window> rwindow4 = gui->addWindow(Eigen::Vector2i(410, 10), "Plot");
    rwindow4->setLayout( new GroupLayout() );
    
    rwindow4->add<Label>("Altitude plot", "sans-bold");
    nanogui::Graph* graph1 = rwindow4->add<nanogui::Graph>("Altitude");
    Eigen::VectorXf& altitude_func = graph1->values();
    altitude_func.resize(MAX_PLOT_SIZE);

    rwindow4->add<Label>("Attitude plot", "sans-bold");
    nanogui::Graph* graph2 = rwindow4->add<nanogui::Graph>("ROLL");
    Eigen::VectorXf& attitude_roll = graph2->values();
    attitude_roll.resize(MAX_PLOT_SIZE);

    nanogui::Graph* graph3 = rwindow4->add<nanogui::Graph>("PITCH");
    Eigen::VectorXf& attitude_pitch = graph3->values();
    attitude_pitch.resize(MAX_PLOT_SIZE);
    
    nanogui::Graph* graph4 = rwindow4->add<nanogui::Graph>("YAW");
    Eigen::VectorXf& attitude_yaw = graph4->values();
    attitude_yaw.resize(MAX_PLOT_SIZE);

    nanogui::ref<Window> rwindow5 = gui->addWindow(Eigen::Vector2i(210, 455), "Video Streamming");
    gui->addGroup("Request and Stop");
    gui->addButton("Video Request", []()
            {
                udp_send_command("VIDEO_REQUEST");

                pthread_create(&receive_id
                        , NULL
                        , receive_video_udp
                        , (void*)NULL);
                DEBUG_MSG("receive_video_udp thread created\n");

                pthread_create(&keep_alive_id
                        , NULL
                        , keep_alive
                        , (void*)NULL);
                DEBUG_MSG("keep_alive thread created\n");
            });
        gui->addButton("Stop", []()
            {
                DEBUG_MSG("request server close and exit\n");
                udp_send_command("QUIT_SERVER");

                set_quit();
                DEBUG_MSG("close and exit client\n");
                udp_client_command_close();
                udp_client_stream_close();
                
                DEBUG_MSG("wait until thread join\n");
                pthread_join(receive_id, (void **)&receive_status);
                pthread_join(keep_alive_id, (void **)&keep_alive_status);
            });
    
    gui_set_done();
    
    bool quit = false;
    SDL_Event event;
    
    auto pre_t = chrono::high_resolution_clock::now();

    while(!quit)
    {
        SDL_PollEvent(&event);

        const uint8_t *key_state = SDL_GetKeyboardState(NULL);
        static uint8_t pre_key_state[SDL_NUM_SCANCODES] = {SDL_SCANCODE_UNKNOWN,};

        switch(event.type)
        {
            case SDL_QUIT:
                quit = true;
                break;
        }

        if(key_state[SDL_SCANCODE_LEFT])
        {
            cout << "left" << endl;
        }
        if(key_state[SDL_SCANCODE_RIGHT])
        {
            cout << "right" << endl;
        }
        if(key_state[SDL_SCANCODE_UP])
        {
            cout << "up" << endl;
        }
        if(key_state[SDL_SCANCODE_DOWN])
        {
            cout << "down" << endl;
        }

        for(uint16_t i = 0; i < SDL_NUM_SCANCODES; i++)
        {
            if((pre_key_state[i] != key_state[i]) && key_state[i])
            {
                switch(i)
                {
                    case SDL_SCANCODE_A:
                        cout << "arming" <<endl;
                        msp_arm();
                        break;
                    case SDL_SCANCODE_D:
                        cout << "disarming" << endl;
                        msp_disarm();
                        break;
                    case SDL_SCANCODE_I:
                        cout << "show info" << endl;

                        msp_get_att(&att);
                        msp_get_alt(&alt);
                        msp_get_imu(&imu);

                        cout << "attitude info" << endl;
                        cout << "angle1 : " << att.angle[0] << endl;
                        cout << "angle2 : " << att.angle[1] << endl;
                        cout << "heading : " << att.heading << endl;

                        cout << "altitude info" << endl;
                        cout << "EstAlt : " << alt.EstAlt << endl;
                        cout << "vario : " << alt.vario << endl;

                        cout << "accSmooth\n" 
                             << imu.accSmooth[0] << '\n' 
                             << imu.accSmooth[1] << '\n' 
                             << imu.accSmooth[2] << '\n' 
                             << "gyroData\n"
                             << imu.gyroData[0] << '\n'
                             << imu.gyroData[1] << '\n'
                             << imu.gyroData[2] << '\n'
                             << "magADC\n"
                             << imu.magADC[0] << '\n'
                             << imu.magADC[1] << '\n'
                             << imu.magADC[2] << '\n'
                             << "gyroADC\n"
                             << imu.gyroADC[0] << '\n'
                             << imu.gyroADC[1] << '\n'
                             << imu.gyroADC[2] << '\n'
                             << "accADC\n"
                             << imu.accADC[0] << '\n'
                             << imu.accADC[1] << '\n'
                             << imu.accADC[2] << endl;
                        break;                    
                }
            }

            pre_key_state[i] = key_state[i];
        }
        
        auto current_t = chrono::high_resolution_clock::now();

        auto elapsed = chrono::duration_cast<chrono::milliseconds>(current_t - pre_t);

        if(elapsed.count() > 16)
        {
            msp_get_att(&att);
            msp_get_alt(&alt);
            msp_get_imu(&imu);
            msp_get_debug(debug);
            for(uint8_t i = 0; i < MAX_PLOT_SIZE-1; i++)
            {
                altitude_func[i]    = altitude_func[i+1];
                attitude_roll[i]    = attitude_roll[i+1];
                attitude_pitch[i]   = attitude_pitch[i+1];
                attitude_yaw[i]     = attitude_yaw[i+1];
            }
            altitude_func[MAX_PLOT_SIZE-1] = alt.EstAlt/500.0;
            attitude_roll[MAX_PLOT_SIZE-1] = att.angle[0]/2000.0+0.5;
            attitude_pitch[MAX_PLOT_SIZE-1] = att.angle[1]/2000.0+0.5;
            attitude_yaw[MAX_PLOT_SIZE-1] = att.heading/360.0+0.5;
            
            gui->refresh();

            gui_draw(event);
            
            pre_t = chrono::high_resolution_clock::now();
        }
    }

    serial_deinit();
    return 0;
}
