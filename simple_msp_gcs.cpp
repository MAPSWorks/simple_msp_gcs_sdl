#include "./serial_setup/serial_setup.h"
#include "./msp_protocol/msp_protocol.h"
#include "./gui/gui.h"

#include <stdint.h>
#include <iostream>
#include <stdio.h>
#include <string.h>

using namespace std;
using namespace nanogui;

#include <SDL2/SDL.h>

int main()
{
    serial_init();
    msp_init();

    att_t att;
    alt_t alt;
    imu_t imu;

    gui_init();

    FormHelper* gui = gui_get_pointer();

    gui->addGroup("attitude info");
    gui->addVariable("angle0", att.angle[0]);
    gui->addVariable("angle1", att.angle[1]);
    gui->addVariable("heading", att.heading);

    gui->addGroup("altitude info");
    gui->addVariable("EstAlt", alt.EstAlt);
    gui->addVariable("vario", alt.vario);

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

    gui_set_done();
    
    bool quit = false;
    SDL_Event event;

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
        
        msp_get_att(&att);
        msp_get_alt(&alt);
        msp_get_imu(&imu);

        gui_draw(event);
    }

    serial_deinit();
    return 0;
}
