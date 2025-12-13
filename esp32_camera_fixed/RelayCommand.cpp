#include "RelayCommand.h"

static HardwareSerial* relayUART = nullptr;

void relayInit(HardwareSerial &uart) {
    relayUART = &uart;
}

void relayOn() {
    if (relayUART) {
        relayUART->write(RELAY_ON_CMD);
        Serial.println("UART: RELAY ON sent");
    }
}

void relayOff() {
    if (relayUART) {
        relayUART->write(RELAY_OFF_CMD);
        Serial.println("UART: RELAY OFF sent");
    }
}
