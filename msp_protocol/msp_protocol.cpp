#include "msp_protocol.h"

#include "../serial_setup/serial_setup.h"

void msp_arm()
{
    serial_write('$');
    serial_write('M');
    serial_write('<');
    serial_write(0);
    serial_write(MSP_ARM);
    serial_write(MSP_ARM);
}

void msp_disarm()
{
    serial_write('$');
    serial_write('M');
    serial_write('<');
    serial_write(0);
    serial_write(MSP_DISARM);
    serial_write(MSP_DISARM);
}
