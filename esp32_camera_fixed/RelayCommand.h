#ifndef RELAY_COMMAND_H
#define RELAY_COMMAND_H

#include <Arduino.h>

#define RELAY_ON_CMD  'R'
#define RELAY_OFF_CMD 'S'

void relayInit(HardwareSerial &uart);
void relayOn();
void relayOff();

#endif
