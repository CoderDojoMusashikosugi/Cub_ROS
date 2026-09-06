#pragma once

#include <Arduino.h>

// Converts a captured micros() value to the synchronized UTC clock.
// Returns false while the Spresense clock is not synchronized.
using UtcAtMicrosFunction = bool (*)(uint32_t captured_micros,
                                     uint64_t* utc_us);

// Configures the UM982 EVENT input and the Spresense GPIO interrupt.
// The input is active LOW: the shared button signal must be connected to GND
// when pressed.
void beginUm982EventComparison(Stream& gnss_serial,
                               uint8_t event_pin,
                               UtcAtMicrosFunction utc_at_micros);

// Pass every byte received from the UM982 to this function.
void feedUm982EventByte(char received);

// Pairs the GPIO capture with EVENTFLAGA and prints the result.
void updateUm982EventComparison();
