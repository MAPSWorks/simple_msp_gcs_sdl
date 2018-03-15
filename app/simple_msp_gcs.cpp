#include "../serial_setup/serial_setup.h"
#include "../msp_protocol/msp_protocol.h"
#include "../gui/gui.h"
#include "../rpi-udp-stream-client/common_util/common_util.h" //use utils from rpi-udp-stream-client submodule
#include "video_stream_part.h"

#include <stdint.h>
#include <iostream>

#include <stdio.h>
#include <string.h>

#include <chrono>
#include <ctime>

using namespace std;
using namespace nanogui;
static FormHelper* gui;

#include <SDL2/SDL.h>

#define MAX_PLOT_SIZE 100

//Drone's information
static att_t att = {9999, };
static alt_t alt = {9999, };
static imu_t imu = {9999, };
static msp_status_t msp_status = {0, };
static uint8_t is_armed = 0;
static uint8_t is_baromode = 0;
static int16_t debug[4] = {9999, };

static int16_t throttle_val = 0;

static Eigen::VectorXf* altitude_func_ptr;
static Eigen::VectorXf* attitude_roll_ptr;
static Eigen::VectorXf* attitude_pitch_ptr;
static Eigen::VectorXf* attitude_yaw_ptr;

static char* stream_ip;

static void app_init();
static void app_deinit();

static void sig_int(int arg)
{ 
    set_quit(); //video quit
    app_deinit();

    exit(1);
}

static void app_init()
{
    //signal interrupt
    signal(SIGINT,  sig_int);
    signal(SIGTERM, sig_int);
    signal(SIGQUIT, sig_int);

    serial_init();
    msp_init();
    video_init(stream_ip);

    gui_init();

    gui = gui_get_pointer();

    nanogui::ref<Window> rwindow = gui->addWindow(Eigen::Vector2i(10, 10), "Basic information");
    gui->addGroup("Status info");
    gui->addVariable("Armed", is_armed);
    gui->addVariable("BARO mode", is_baromode);

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

    nanogui::ref<Window> rwindow3 = gui->addWindow(Eigen::Vector2i(10, 475), "Arming");
    gui->addGroup("Arming");
    gui->addButton("Arm", []()
            {
                DEBUG_MSG("Arm\n");
                msp_attitude_input_default();
                throttle_val = 0;
                msp_throttle(throttle_val);
                msp_arm();
            });
    gui->addButton("Disarm", []()
            {
                DEBUG_MSG("Disarm\n");
                msp_attitude_input_default();
                throttle_val = 0;
                msp_throttle(throttle_val);
                msp_disarm();
            });
    gui->addGroup("Calibration");
    gui->addButton("Acc calibration", []()
            {
                DEBUG_MSG("Acc calibration\n");
                msp_acc_calib();
            });
    gui->addButton("Mag calibration", []()
            {
                DEBUG_MSG("Mag calibration\n");
                msp_mag_calib();
            });
    gui->addGroup("Save setting");
    gui->addButton("EEPROM write", []()
            {
                DEBUG_MSG("EEPROM write\n");
                msp_eeprom_write();
            });

    nanogui::ref<Window> rwindow4 = gui->addWindow(Eigen::Vector2i(410, 10), "Plot");
    rwindow4->setLayout( new GroupLayout() );

    rwindow4->add<Label>("Altitude plot", "sans-bold");
    nanogui::Graph* graph1 = rwindow4->add<nanogui::Graph>("Altitude");
    Eigen::VectorXf& altitude_func = graph1->values();
    altitude_func.resize(MAX_PLOT_SIZE);
    altitude_func_ptr = &altitude_func;

    rwindow4->add<Label>("Attitude plot", "sans-bold");
    nanogui::Graph* graph2 = rwindow4->add<nanogui::Graph>("ROLL");
    Eigen::VectorXf& attitude_roll = graph2->values();
    attitude_roll.resize(MAX_PLOT_SIZE);
    attitude_roll_ptr = &attitude_roll;

    nanogui::Graph* graph3 = rwindow4->add<nanogui::Graph>("PITCH");
    Eigen::VectorXf& attitude_pitch = graph3->values();
    attitude_pitch.resize(MAX_PLOT_SIZE);
    attitude_pitch_ptr = &attitude_pitch;

    nanogui::Graph* graph4 = rwindow4->add<nanogui::Graph>("YAW");
    Eigen::VectorXf& attitude_yaw = graph4->values();
    attitude_yaw.resize(MAX_PLOT_SIZE);
    attitude_yaw_ptr = &attitude_yaw;

    nanogui::ref<Window> rwindow5 = gui->addWindow(Eigen::Vector2i(210, 455), "Video Streamming");
    gui->addGroup("Request and Stop");
    gui->addButton("Video Request", []()
            {
                DEBUG_MSG("Start video streamming\n");
                video_start();
            });
    gui->addButton("Stop", []()
            {
                DEBUG_MSG("Stop video Streamming\n");
                video_stop();
            });

    gui_set_done();
}

static void app_deinit()
{
    video_deinit();
    serial_deinit();
}

int main(int argc, char* argv[])
{
    if(argc != 2) 
    {
        DEBUG_ERR("Usage: %s IP_address\n", argv[0]);
        exit(1);
    }
    stream_ip = argv[1]; //get ip address of video server

    app_init();
    Eigen::VectorXf& altitude_func = *altitude_func_ptr;
    Eigen::VectorXf& attitude_roll = *attitude_roll_ptr;
    Eigen::VectorXf& attitude_pitch = *attitude_pitch_ptr;
    Eigen::VectorXf& attitude_yaw = *attitude_yaw_ptr;

    SDL_Event event;

    auto pre_display_t = chrono::high_resolution_clock::now();

    while(!is_quit())
    {
        SDL_PollEvent(&event);

        const uint8_t *key_state = SDL_GetKeyboardState(NULL);
        static uint8_t pre_key_state[SDL_NUM_SCANCODES] = {SDL_SCANCODE_UNKNOWN,};

        switch(event.type)
        {
            case SDL_QUIT:
                set_quit();
                break;
        }
        
        static uint16_t wheel_count = 0;
        if(event.type == SDL_MOUSEWHEEL)
        {
            wheel_count++;
            if(wheel_count > 10000)
            {
                DEBUG_MSG("wheel direction : %d\n", event.wheel.y);
                if(event.wheel.y == 1) // scroll up
                {
                    DEBUG_MSG("mouse wheel up\n");
                    throttle_val += 1;        
                }
                else if(event.wheel.y == -1) // scroll down
                {
                    DEBUG_MSG("mouse wheel down\n");
                    throttle_val -= 1;
                }
                if(throttle_val > 800)
                    throttle_val = 800;
                if(throttle_val < 0)
                    throttle_val = 0;
                msp_throttle(throttle_val);
                
                wheel_count = 0;
            }
        }

        if(!(key_state[SDL_SCANCODE_A]
             ||key_state[SDL_SCANCODE_D]
             ||key_state[SDL_SCANCODE_W]
             ||key_state[SDL_SCANCODE_S]))
        {
            msp_attitude_input_default();
        }
        else
        {
            if(key_state[SDL_SCANCODE_A])
            {
                DEBUG_MSG("left\n");
                msp_left();
            }
            if(key_state[SDL_SCANCODE_D])
            {
                DEBUG_MSG("right\n");
                msp_right();
            }
            if(key_state[SDL_SCANCODE_W])
            {
                DEBUG_MSG("up\n");
                msp_forward();
            }
            if(key_state[SDL_SCANCODE_S])
            {
                DEBUG_MSG("donw\n");
                msp_backward();
            }
        }
        for(uint16_t i = 0; i < SDL_NUM_SCANCODES; i++)
        {
            if((pre_key_state[i] != key_state[i]) && key_state[i])
            {
                switch(i)
                {
                    case SDL_SCANCODE_Q:
                        DEBUG_MSG("Go to ALT HOLD mode\n");
                        msp_set_alt_mod();
                        break;
                    case SDL_SCANCODE_E:
                        DEBUG_MSG("END ALT HOLD Mode\n");
                        msp_reset_alt_mod();
                        break;
                }
            }

            pre_key_state[i] = key_state[i];
        }

        auto current_display_t = chrono::high_resolution_clock::now();

        auto display_elapsed = chrono::duration_cast<chrono::milliseconds>(current_display_t - pre_display_t);

        if(display_elapsed.count() > 16)
        {
            msp_get_att(&att);
            msp_get_alt(&alt);
            msp_get_imu(&imu);
            msp_get_status(&msp_status);
            is_armed = (msp_status.flag >> BOXARM) & 0x01;
            is_baromode = (msp_status.flag >> BOXBARO) & 0x01;
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

            pre_display_t = chrono::high_resolution_clock::now();
        }
    }

    app_deinit();

    return 0;
}
