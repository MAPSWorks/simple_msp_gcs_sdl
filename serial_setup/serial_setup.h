#ifndef SERIAL_SETUP_H
#define SERIAL_SETUP_H

#include <stdint.h>

void serial_init();
void serial_deinit();

void serial_write(uint8_t chr);
uint8_t serial_read();

int serial_available();

#endif
