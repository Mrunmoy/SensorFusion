#pragma once

#include "SFRegistry.hpp"
#include "TaskRate.hpp"

struct EcgCtx
{
	float hz = 250.0f; // sampling rate
	// add pointers here later if we want registry/websocket, etc.
};

// Forward decl for FreeRTOS task signature
extern "C" void EcgTask(void *pv);

extern "C" void EcgSetNotchHz(float hz);
extern "C" void EcgSetNotchQ(float q);