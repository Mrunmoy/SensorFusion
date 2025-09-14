#pragma once

#include "SFRegistry.hpp"
#include "TaskRate.hpp"

// Forward-declare your drivers
class MPU6050Driver;
class QMC5883LDriver;
class BMP180Driver;

struct ImuCtx
{
	SFReg *reg;
	MPU6050Driver *mpu;
	float hz;
};

struct MagCtx
{
	SFReg *reg;
	QMC5883LDriver *mag;
	float hz;
};

struct BaroCtx
{
	SFReg *reg;
	BMP180Driver *baro;
	float hz;
};

struct FusionCtx
{
	SFReg *reg;
	float hz;
};

// FreeRTOS-compatible task entry points
extern "C" void ImuTask(void *pv);
extern "C" void MagTask(void *pv);
extern "C" void BaroTask(void *pv);
extern "C" void FusionTask(void *pv);
