#pragma once

#include <Arduino.h>

// CAN client task for node-side MCU (TWAI + MCP2561)
void canNodeTask(void *);
void canNodeInit();
