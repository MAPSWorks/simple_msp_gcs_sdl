#include "./serial_setup/serial_setup.h"
#include "./msp_protocol/msp_protocol.h"

#include <iostream>
#include <stdio.h>
#include <string.h>
using namespace std;

int main()
{
    serial_init();
    
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
            else if(!strncmp("q", user_input, 1))
            {
                break;
            }
        }
    }

    serial_deinit();
    return 0;
}
