#pragma once

#include <Arduino.h>

// Serial <-> CAN bridge task (TWAI + MCP2561 transceiver)
void canBridgeTask(void *);
void canBridgeInit();
