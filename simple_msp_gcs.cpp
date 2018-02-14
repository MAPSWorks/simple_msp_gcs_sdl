#include "./serial_setup/serial_setup.h"
#include "./msp_protocol/msp_protocol.h"

#include <iostream>
#include <stdio.h>
#include <string.h>
using namespace std;

int main()
{
    serial_init();
    msp_init();

    att_t att;
    alt_t alt;
    
    while(1)
    {
        char user_input[128];
        if(fgets(user_input, 128, stdin) != NULL)
        {
            if(!strncmp("a", user_input, 1))
            {
                cout << "arming" << endl;
                msp_arm();
            }
            else if(!strncmp("d", user_input, 1))
            {
                cout << "disarming" << endl;
                msp_disarm();
            }
            else if(!strncmp("i", user_input, 1))
            {
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
            }
            else if(!strncmp("q", user_input, 1))
            {
                break;
            }
        }
    }

    serial_deinit();
    return 0;
}
