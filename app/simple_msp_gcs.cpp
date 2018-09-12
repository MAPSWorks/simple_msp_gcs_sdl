#include "../serial_setup/serial_setup.h"
#include "../msp_protocol/msp_protocol.h"
#include "../gui/gui.h"
#include "../rpi-udp-stream-client/common_util/common_util.h" //use utils from rpi-udp-stream-client submodule
#include "../rpi-udp-stream-client/computer_vision/get_optical_flow.h"
#include "video_stream_part.h"
#include "flow_mode.h"
#include "video_log.h"

#include <stdint.h>
#include <iostream>
#include <fstream>

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
static drone_info_t drone_info = {0, };
static opt_flow_t flow;
static int16_t flow_mode_output[2];
static bool flow_mode_state = 0;

//Throttle control mode
static int16_t throttle_val = 0;
static bool is_auto_takeoff = false;

static Eigen::VectorXf* altitude_func_ptr;
static Eigen::VectorXf* attitude_roll_ptr;
static Eigen::VectorXf* attitude_pitch_ptr;
static Eigen::VectorXf* attitude_yaw_ptr;

static char* stream_ip;

static bool log_state = 0;
ofstream log_file;

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
    gui->addVariable("Armed", drone_info.arm_status);
    gui->addVariable("BARO mode", drone_info.baro_mode_status);

    gui->addGroup("attitude info");
    gui->addVariable("angle0", drone_info.angle[0]);
    gui->addVariable("angle1", drone_info.angle[1]);
    gui->addVariable("heading", drone_info.heading);

    gui->addGroup("altitude info");
    gui->addVariable("EstAlt", drone_info.height);

    drone_info.debug[0] = 9999; //just for text box size in gui
    gui->addGroup("Debug Data");
    gui->addVariable("Debug[0]", drone_info.debug[0]);
    gui->addVariable("Debug[1]", drone_info.debug[1]);
    gui->addVariable("Debug[2]", drone_info.debug[2]);
    gui->addVariable("Debug[3]", drone_info.debug[3]);

    nanogui::ref<Window> rwindow2 = gui->addWindow(Eigen::Vector2i(210, 10), "Raw sensor data");
    drone_info.accSmooth[0] = 9999;
    gui->addGroup("Raw sensor data"); //just for text box size in gui
    gui->addVariable("accSmooth 0", drone_info.accSmooth[0]);
    gui->addVariable("accSmooth 1", drone_info.accSmooth[1]);
    gui->addVariable("accSmooth 2", drone_info.accSmooth[2]);

    gui->addVariable("gyroData 0", drone_info.gyroData[0]);
    gui->addVariable("gyroData 1", drone_info.gyroData[1]);
    gui->addVariable("gyroData 2", drone_info.gyroData[2]);

    nanogui::ref<Window> rwindow3 = gui->addWindow(Eigen::Vector2i(700, 10), "Arming");
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
    gui->addGroup("Trim");
    gui->addButton("Trim Up", []()
            {
                DEBUG_MSG("Trim Up\n");
                msp_set_trim_up();
            });
    gui->addButton("Trim Down", []()
            {
                DEBUG_MSG("Trim Down\n");
                msp_set_trim_down();
            });
    gui->addButton("Trim Left", []()
            {
                DEBUG_MSG("Trim Left\n");
                msp_set_trim_left();
            });
    gui->addButton("Trim Right", []()
            {
                DEBUG_MSG("Trim Right\n");
                msp_set_trim_right();
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

    nanogui::ref<Window> rwindow5 = gui->addWindow(Eigen::Vector2i(520, 323), "Video Streamming");
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

    nanogui::ref<Window> rwindow6 = gui->addWindow(Eigen::Vector2i(210, 240), "RC status");
    gui->addGroup("RC state of drone");
    drone_info.rcData[0] = 9999; // just for text box size 
    gui->addVariable("Roll"     , drone_info.rcData[0]);
    gui->addVariable("Pitch"    , drone_info.rcData[1]);
    gui->addVariable("Yaw"      , drone_info.rcData[2]);
    gui->addVariable("Throttle" , drone_info.rcData[3]);
    gui->addVariable("AUX1"     , drone_info.rcData[4]);

    nanogui::ref<Window> rwindow7 = gui->addWindow(Eigen::Vector2i(880, 10), "Logging");
    gui->addGroup("Logging");
    gui->addButton("Log start", []()
            {
                DEBUG_MSG("Start logging\n");
                log_file.open("log_file.txt", 	ios::out|ios::trunc);
                log_state = 1;
            });
    gui->addButton("Log stop", []()
            {
                DEBUG_MSG("Stop logging\n");
                log_state = 0;
                log_file.close();
            });
    gui->addButton("Log video start", []()
            {
                DEBUG_MSG("Start logging with video\n");
                video_log_init();
            });
    gui->addButton("Log video stop", []()
            {
                DEBUG_MSG("Stop logging with video\n");
                video_log_deinit();
            });

    nanogui::ref<Window> rwindow8 = gui->addWindow(Eigen::Vector2i(880, 200), "Flow mode");
    gui->addGroup("Flow mode");
    gui->addButton("Flow mode start", []()
            {
                DEBUG_MSG("Start Flow mode\n");
                flow_mode_state = 1;
            });
    gui->addButton("Flow mode stop", []()
            {
                DEBUG_MSG("Stop Flow mode\n");
                flow_mode_state = 0;
            });

    nanogui::ref<Window> rwindow9 = gui->addWindow(Eigen::Vector2i(880, 420), "Auto TakeOff mode");
    gui->addGroup("Auto TakeOff");
    gui->addButton("TakeOff start", []()
            {
                DEBUG_MSG("Start TakeOff\n");
                is_auto_takeoff = true;
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
    uint8_t sdl_event_state = 0;

    auto pre_drone_info_t = chrono::high_resolution_clock::now();
    auto pre_takeoff_t = chrono::high_resolution_clock::now();
    auto pre_display_t = chrono::high_resolution_clock::now();
    auto pre_log_t = chrono::high_resolution_clock::now();
    auto pre_flow_t = chrono::high_resolution_clock::now();

    while(!is_quit())
    {
        // Get poll event and state
        sdl_event_state = SDL_PollEvent(&event);

        const uint8_t *key_state = SDL_GetKeyboardState(NULL);
        static uint8_t pre_key_state[SDL_NUM_SCANCODES] = {SDL_SCANCODE_UNKNOWN,};

        // NanoGUI display
        auto current_drone_info_t = chrono::high_resolution_clock::now();
        auto drone_info_elapsed = chrono::duration_cast<chrono::milliseconds>(current_drone_info_t - pre_drone_info_t);

        if(drone_info_elapsed.count() > 6)
        {
            msp_get_info(&drone_info);
            pre_drone_info_t = chrono::high_resolution_clock::now();
        }

        //Automatically takeoff 
        if(is_auto_takeoff)
        {
            auto current_takeoff_t = chrono::high_resolution_clock::now();
            auto takeoff_elapsed = chrono::duration_cast<chrono::milliseconds>(current_takeoff_t - pre_takeoff_t);

            if(takeoff_elapsed.count() > 10)
            {
                //automatically set throttle
                //higher than mid-point
                if(drone_info.height < 10)
                {
                    throttle_val++;
                    if(throttle_val > 175)
                        throttle_val = 175;
                    msp_throttle(throttle_val);
                }

                if(drone_info.height >= 100)
                {
                    DEBUG_MSG("Go to ALT HOLD mode\n");
                    if(!drone_info.baro_mode_status)
                        msp_set_alt_mod();
                    else
                        is_auto_takeoff = false;
                }

                pre_takeoff_t = chrono::high_resolution_clock::now();
            }
        }
        else
        {
            // Event driven loop
            if(sdl_event_state)
            {
                switch(event.type)
                {
                    case SDL_QUIT:
                        set_quit();
                        break;
                }
                if(event.type == SDL_MOUSEWHEEL)
                {
                    if(event.wheel.y == 1) // scroll up
                    {
                        DEBUG_MSG("mouse wheel up\n");
                        throttle_val += 3;
                    }
                    else if(event.wheel.y == -1) // scroll down
                    {
                        DEBUG_MSG("mouse wheel down\n");
                        throttle_val -= 3;
                    }
                    if(throttle_val > 250)
                        throttle_val = 250;
                    if(throttle_val < 0)
                        throttle_val = 0;
                    msp_throttle(throttle_val);
                    DEBUG_MSG("Throttle value : %d\n", throttle_val);
                }
            }
        }
        
        // Direct Key input handle
        if(!(key_state[SDL_SCANCODE_A]
                    ||key_state[SDL_SCANCODE_D]
                    ||key_state[SDL_SCANCODE_W]
                    ||key_state[SDL_SCANCODE_S]
                    ||key_state[SDL_SCANCODE_Z]
                    ||key_state[SDL_SCANCODE_C]))
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
                DEBUG_MSG("down\n");
                msp_backward();
            }
            if(key_state[SDL_SCANCODE_Z])
            {
                DEBUG_MSG("turn left\n");
                msp_turn_left();
            }
            if(key_state[SDL_SCANCODE_C])
            {
                DEBUG_MSG("turn right\n");
                msp_turn_right();
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
       
        // NanoGUI display
        auto current_display_t = chrono::high_resolution_clock::now();
        auto display_elapsed = chrono::duration_cast<chrono::milliseconds>(current_display_t - pre_display_t);

        if(display_elapsed.count() > 16)
        {
            msp_get_info(&drone_info);
            for(uint8_t i = 0; i < MAX_PLOT_SIZE-1; i++)
            {
                altitude_func[i]    = altitude_func[i+1];
                attitude_roll[i]    = attitude_roll[i+1];
                attitude_pitch[i]   = attitude_pitch[i+1];
                attitude_yaw[i]     = attitude_yaw[i+1];
            }
            altitude_func[MAX_PLOT_SIZE-1] = drone_info.height/500.0;
            attitude_roll[MAX_PLOT_SIZE-1] = drone_info.angle[0]/2000.0+0.5;
            attitude_pitch[MAX_PLOT_SIZE-1] = drone_info.angle[1]/2000.0+0.5;
            attitude_yaw[MAX_PLOT_SIZE-1] = drone_info.heading/360.0+0.5;

            gui->refresh();

            gui_draw(event);

            pre_display_t = chrono::high_resolution_clock::now();
        }

        // flow mode
        auto current_flow_t = chrono::high_resolution_clock::now();
        auto flow_elapsed = chrono::duration_cast<chrono::milliseconds>(current_flow_t - pre_flow_t);

        if(flow_elapsed.count() >20)
        {
            if(flow_mode_state)
            {
                get_opt_flow_data(&flow);

                flow_mode_set_flow(flow.output_point.x, flow.output_point.y);
                flow_mode_set_gyro(drone_info.gyroData[0], drone_info.gyroData[1]);
                flow_mode_set_altitude(drone_info.height);

                do_flow_mode();

                get_flow_output(&flow_mode_output[0], &flow_mode_output[1]);
                msp_set_flow_output(flow_mode_output[0], flow_mode_output[1]);
            }
            else
                msp_set_flow_output(0, 0);

            pre_flow_t = chrono::high_resolution_clock::now();
        }

        // loggging
        auto current_log_t = chrono::high_resolution_clock::now();
        auto log_elapsed = chrono::duration_cast<chrono::milliseconds>(current_log_t - pre_log_t);

        if(log_elapsed.count() >20)
        {
            if(log_state)
            {
                log_file //<< drone_info.gyroData[0] 
                         //<< '\t' << drone_info.gyroData[1] 
                         //<< '\t' << drone_info.gyroData[2] 
                         //<< '\t' << flow.output_point.x
                         //<< '\t' << flow.output_point.y
                         << '\t' << flow_mode_output[0]
                         << '\t' << flow_mode_output[1]
                         << '\t' << drone_info.height
                         << '\t' << endl;
            }
            pre_log_t = chrono::high_resolution_clock::now();
        }
    }

    app_deinit();

    return 0;
}
