#pragma once

#include "device.h"

// Call this to trigger sending of Vtx packet
void VtxTriggerSend();
void VtxPitmodeSwitchUpdate();
uint8_t VtxGetCurrentChannel();
uint8_t VtxGetCurrentBand();
uint8_t VtxGetCurrentPower();
extern device_t VTX_device;