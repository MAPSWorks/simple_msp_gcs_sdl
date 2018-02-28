#include "./serial_setup/serial_setup.h"
#include "./msp_protocol/msp_protocol.h"

#include <stdint.h>
#include <iostream>
#include <stdio.h>
#include <string.h>

using namespace std;

#include <SDL2/SDL.h>

int main()
{
    serial_init();
    msp_init();

    SDL_Init(SDL_INIT_VIDEO);

    SDL_Window * window = SDL_CreateWindow("SDL2 Keyboard/Mouse events"
            , SDL_WINDOWPOS_UNDEFINED
            , SDL_WINDOWPOS_UNDEFINED
            , 640
            , 480
            , 0);

    att_t att;
    alt_t alt;

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

                        cout << "attitude info" << endl;
                        cout << "angle1 : " << att.angle[0] << endl;
                        cout << "angle2 : " << att.angle[1] << endl;
                        cout << "heading : " << att.heading << endl;

                        cout << "altitude info" << endl;
                        cout << "EstAlt : " << alt.EstAlt << endl;
                        cout << "vario : " << alt.vario << endl;
                        break;                    
                }
            }

            pre_key_state[i] = key_state[i];
        }
    }

    SDL_DestroyWindow(window);
    serial_deinit();
    return 0;
}
