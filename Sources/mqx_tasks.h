// Copyright (c) 2014, Freescale Semiconductor, Inc.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Freescale Semiconductor, Inc. nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL FREESCALE SEMICONDUCTOR, INC. BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#ifndef MQX_TASKS_H
#define MQX_TASKS_H

#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "Cpu.h"
#include "MQX1.h"
#include "SystemTimer1.h"
#include "LED_RED.h"
#include "LED_GREEN.h"
#include "LED_BLUE.h"
#include "FTM.h"
#include "UART.h"
#include "I2C.h"
#include "TestPin_KF_Time.h"
#include "TestPin_MagCal_Time.h"
#include "FTM.h"
#include "UART.h"
#include "lwevent.h"
#include "build.h"

// project globals structure
struct ProjectGlobals
{
	// bluetooth flags
	uint8 AngularVelocityPacketOn;
	uint8 DebugPacketOn;
	uint8 RPCPacketOn;
	uint8 AltPacketOn;

	// bluetooth quaternion type
	quaternion_type QuaternionPacketType;
	
	// default quaternion type
	quaternion_type DefaultQuaternionPacketType;

	// magnetic packet variable identifier
	int16 MagneticPacketID;

	// packet number
	uint8 iPacketNumber;  	

	// MPL3115 found flag
	int8 iMPL3115Found; 
	
	// global counter incrementing each iteration of sensor fusion (typically 25Hz)
	int32 loopcounter;
};

// mqx-lite globals structure
struct MQXLiteGlobals
{
	// MQX-Lite Events
	LWEVENT_STRUCT SamplingEventStruct;	// hardware timer event
	LWEVENT_STRUCT RunKFEventStruct;	// kalman filter sensor fusion event
	LWEVENT_STRUCT MagCalEventStruct;	// magnetic calibration event

	// FTM hardware time globals
	uint16 FTMReload;               	// FTM wraparound reload value
	uint32 FTMTimestamp ;   			// time stamp updated from FTM

	// flags
	volatile uint8 I2C_Status;
	volatile uint16 RunKF_Event_Flag;
	volatile uint16 MagCal_Event_Flag;
};

// globals defined in mqx_tasks.c
extern struct ProjectGlobals globals;
extern struct MQXLiteGlobals mqxglobals;

// function prototypes for functions in mqx_tasks.c
void Main_task(uint32_t task_init_data);
void RdSensData_task(uint32_t task_init_data);
void Fusion_task(uint32_t task_init_data);
void MagCal_task(uint32_t task_init_data);

#endif // MQX_TASKS_H
