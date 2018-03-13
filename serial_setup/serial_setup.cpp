#include "serial_setup.h"

#include <stdio.h>
#include <unistd.h>
#include <sstream>
#include <iostream>
#include <fcntl.h>          //Used for UART
#include <termios.h>        //Used for UART
#include <sys/ioctl.h>      //Used for UART

using namespace std;

#include "../rpi-udp-stream-client/common_util/common_util.h"

static string found_device;
static int serial_device_file = -1;

void serial_init()
{
    FILE* stream = popen( "dmesg | grep \"attached to tty\"", "r" );

    ostringstream output;

    DEBUG_MSG("Please detach and attach Serial Device\n");

    //check dmesg to find new device
    while(!feof(stream) && !ferror(stream))
    {
        char buf[128];
        int bytesRead = fread(buf, 1, 128, stream);
        output.write(buf, bytesRead);
    }
    string pre_state = output.str();

    pclose(stream);

    while(1)
    {
        FILE* stream = popen( "dmesg | grep \"attached to tty\"", "r" );

        output.str("");
        output.clear();
        
        while(!feof(stream) && !ferror(stream))
        {
            char buf[128];
            int bytesRead = fread(buf, 1, 128, stream);
            output.write(buf, bytesRead);
        }
        string current_state = output.str();

        pclose(stream);
        
        //something changed, check tty* device and parse
        if(current_state.length() != pre_state.length())
        {
            found_device = current_state.substr(pre_state.length()
                                                , current_state.length()
                                                  -pre_state.length());
            if(found_device.find("tty") != string::npos)
            {
                int pos = pre_state.length()+found_device.find("tty");
 
                found_device = current_state.substr(pos, current_state.length()
                                                         -pre_state.length());
                istringstream istr(found_device);
                istr >> found_device;
                found_device = "/dev/" + found_device;
            }
            break;
        }
        usleep(100000);
    }

    //device find process done
    DEBUG_MSG("Device found : %s\n", found_device.c_str());
    sleep(3);

    //OPEN THE UART
    //The flags (defined in fcntl.h):
    //  Access modes (use 1 of these):
    //      O_RDONLY - Open for reading only.
    //      O_RDWR - Open for reading and writing.
    //      O_WRONLY - Open for writing only.
    //
    //  O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //                                          if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //                                          immediately with a failure status if the output can't be written immediately.
    //
    //  O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
    serial_device_file = open(found_device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);      //Open in non blocking read/write mode
    if(serial_device_file == -1)
    {
        //ERROR - CAN'T OPEN SERIAL PORT
        DEBUG_MSG("Error - Unable to open UART.  Ensure it is not in use by another application");
    }    //CONFIGURE THE UART
    //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    //  Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //  CSIZE:- CS5, CS6, CS7, CS8
    //  CLOCAL - Ignore modem status lines
    //  CREAD - Enable receiver
    //  IGNPAR = Ignore characters with parity errors
    //  ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //  PARENB - Parity enable
    //  PARODD - Odd parity (else even)
    struct termios options;
    tcgetattr(serial_device_file, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;       //<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(serial_device_file, TCIFLUSH);
    tcsetattr(serial_device_file, TCSANOW, &options);

    DEBUG_MSG("serial initialise done\n");
}

void serial_deinit()
{
    //----- CLOSE THE UART -----
    close(serial_device_file);

    DEBUG_MSG("serial deinitialise done\n");
}

void serial_write(uint8_t chr)
{
    write(serial_device_file, &chr, 1);
}

uint8_t serial_read()
{
    uint8_t rx_buffer;
    read(serial_device_file, (void*)&rx_buffer, 1);
    return rx_buffer;
}

int serial_available()
{
    int bytes_available;
    ioctl(serial_device_file, FIONREAD, &bytes_available);
    return bytes_available;
}
