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
// This file defines real-time tasks required by the sensor fusion application.
// These are:
// * main task (one time startup)
// * sampling task (200 Hz nominal)
// * sensor fusion task (25Hz nominal)
// * magnetic calibration (background)
// These functions are intertwined with MQXLITE.  If you change RTOSs, you will have
// to rework this file.
//
#include "Cpu.h"
#include "Events.h"
#include "mqx_tasks.h"

/* User includes (#include below this line is not maintained by Processor Expert) */
#include <stdio.h>
#include "LED_RED.h"
#include "LED_GREEN.h"
#include "LED_BLUE.h"
#include "I2C.h"
#include "UART.h"
#include "FTM.h"
#include "include_all.h"

// MQX-Lite globals structure
struct ProjectGlobals globals;
struct MQXLiteGlobals mqxglobals;

// Main task
void Main_task(uint32_t task_init_data)
{
	// switch the red LED on (line low sets LED on)
	LED_RED_ClrVal(NULL);
	LED_GREEN_SetVal(NULL);
	LED_BLUE_SetVal(NULL);

	// create the sensor sampling event (typically 200Hz)
	_lwevent_create(&(mqxglobals.SamplingEventStruct), LWEVENT_AUTO_CLEAR);
	// create the Kalman filter sensor fusion event (typically 25Hz)
	_lwevent_create(&(mqxglobals.RunKFEventStruct), LWEVENT_AUTO_CLEAR);
	// create the magnetic calibration event (typically once per minute)
	_lwevent_create(&(mqxglobals.MagCalEventStruct), LWEVENT_AUTO_CLEAR);

	// create the sensor read task (controlled by sensor sampling event SamplingEventStruct)	
	_task_create_at(0, RDSENSDATA_TASK, 0, RdSensData_task_stack, RDSENSDATA_TASK_STACK_SIZE);
	// create the sensor fusion task (controlled by sensor fusion event RunKFEventStruct)	
	_task_create_at(0, FUSION_TASK, 0, Fusion_task_stack, FUSION_TASK_STACK_SIZE);
	// create the magnetic calibration task (controlled by MagCalEventStruct)	
	_task_create_at(0, MAGCAL_TASK, 0, MagCal_task_stack, MAGCAL_TASK_STACK_SIZE);
	// and this main task uses about 512 bytes stack for a grand total of 3K task stack space

	// set the sensor sampling frequency (typically 200Hz)
	// this is set to 200Hz by default in PE but we want to set it using value in proj_config.h 
	FTM_SetPeriodTicks(FTM_DeviceData, (uint16) (FTM_INCLK_HZ / SENSORFS));
	
	// initialize globals
	mqxglobals.FTMReload = (uint16)(FTM_INCLK_HZ / SENSORFS);
	mqxglobals.FTMTimestamp = 0;
	globals.iPacketNumber = 0;
	globals.AngularVelocityPacketOn = true;
	globals.DebugPacketOn = true;
	globals.RPCPacketOn = true;
	globals.AltPacketOn = true;
	globals.iMPL3115Found = false;
	globals.MagneticPacketID = 0;

	// initialize the BlueRadios Bluetooth module and other user tasks
	UserStartup();
	
	// initialize the incoming command buffer to all '~' = 0x7E and trigger a callback 
	// when any single command character is received into the UART buffer
	iCommand[0] = iCommand[1] = iCommand[2] = iCommand[3] = '~';
	UART_ReceiveBlock(UART_DeviceData, sUARTInputBuf, 1);

	// destroy this task (main task) now that the three new tasks are created
	_task_destroy(MQX_NULL_TASK_ID);

	return;
}

// sensor read task
void RdSensData_task(uint32_t task_init_data)
{
	// initialize the physical sensors over I2C and the sensor data structures
	RdSensData_Init();

	// initialize the user high frequency (typically 200Hz) task
	UserHighFrequencyTaskInit();

	// loop indefinitely (typically 200Hz)
	while(1)
	{
		// wait here for the sampling event (hardware clock, typically at 200Hz)
		// the Kalman filter and magnetic fusion tasks execute while this task is blocked here
		// FALSE means any bit (of the 1 bit enabled by the mask) unblocks
		// and NULL means timeout is infinite
		_lwevent_wait_for(&(mqxglobals.SamplingEventStruct), 1, FALSE, NULL);

		// reset the Kalman filter task flag
		mqxglobals.RunKF_Event_Flag = 0;
		// read the sensors
		RdSensData_Run();
		// run the user high frequency task
		UserHighFrequencyTaskRun();

		// use the Kalman filter flag set by the sensor read task (typically every 8 iterations)
		// with a mask of 1 (least significant bit set) to enable the Kalman filter task to run
		if (mqxglobals.RunKF_Event_Flag)
			_lwevent_set(&(mqxglobals.RunKFEventStruct), 1);

	}  // end of infinite loop
}

// Kalman filter sensor fusion task
void Fusion_task(uint32_t task_init_data)
{
	uint16_t LedGreenCounter = 0;

	// initialize the sensor fusion algorithms
	Fusion_Init();

	// initialize the user medium frequency (typically 25Hz) task
	UserMediumFrequencyTaskInit();

	// infinite loop controlled by MQX-Lite events
	while(1)
	{
		// ensure the red LED (power up check) is off (line high)
		LED_RED_SetVal(NULL);
		
		// set the output test pin to zero (for timing measurements)
		TestPin_KF_Time_ClrVal(NULL);

		// wait for the sensor fusion event to occur
		// FALSE means any bit (of the 1 bit enabled by the mask) unblocks
		// and NULL means timeout is infinite
		_lwevent_wait_for(&(mqxglobals.RunKFEventStruct), 1, FALSE, NULL);

		// set the output test pin to high to denote sensor fusion running
		TestPin_KF_Time_SetVal(NULL);

		// flash the green LED to denote the sensor fusion is running
		if (++LedGreenCounter >= 5)
		{
			LED_GREEN_NegVal(NULL);
			LedGreenCounter = 0;
		}

		// reset the magnetic calibration flag (this is set by the fusion algorithm)
		mqxglobals.MagCal_Event_Flag = 0;
		// call the sensor fusion algorithms
		Fusion_Run();

		// run the user medium frequency (typically 25Hz) user task
		UserMediumFrequencyTaskRun();

		// enable the magnetic calibration event if the flag for a new magnetic calibration
		// with a mask of 1 (least significant bit set)
		// was set by the sensor fusion algorithms
		if (mqxglobals.MagCal_Event_Flag)
		{
			_lwevent_set(&(mqxglobals.MagCalEventStruct), 1);
		}

	} // end of infinite loop

	return;
}

// magnetic calibration task
void MagCal_task(uint32_t task_init_data)
{
	while(1)
	{
		// set the RED LED off and set test pin off ready for magnetic calibration to run
		LED_RED_SetVal(NULL);           
		TestPin_MagCal_Time_ClrVal(NULL);  

		// wait for the magnetic calibration event
		// this event will never be enabled for build options which don't require magnetic calibration
		// FALSE means any bit (of the 1 bit enabled by the mask) unblocks
		// and NULL means timeout is infinite
		_lwevent_wait_for(&(mqxglobals.MagCalEventStruct), 1, FALSE, NULL);

		// set the red LED on and test pin pin on
		LED_RED_ClrVal(NULL);            
		TestPin_MagCal_Time_SetVal(NULL);  

		// prevent compilation errors when magnetic calibration is not required
#if defined COMPUTE_6DOF_GB_BASIC || defined COMPUTE_9DOF_GBY_KALMAN
		// and run the magnetic calibration
		MagCal_Run(&thisMagCal, &thisMagBuffer);
#endif

	} // end of infinite loop

	return;
}
