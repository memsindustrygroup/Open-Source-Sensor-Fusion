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
// If you want to take a simplistic approach of using the existing templates
// as your starting point, simply adding your code for startup, sampling task (eg. 200Hz)
// and fusion tasks (eg. 25Hz), then this is the file for you.  Just put your code
// where ever you see "PUT YOUR CODE HERE"
//
#include "Cpu.h"
#include "Events.h"
#include "mqx_tasks.h"
#include "UART.h"
#include "include_all.h"

void UserStartup(void)
{
	// The following UART functions initialize Bluetooth communications used by the
	// Freescale Sensor Fusion Toolbox.  If the developer is not using the toolbox,
	// these can be removed.
	//
	// initialize BlueRadios Bluetooth module
	BlueRadios_Init(UART_DeviceData);

	// put code here to be executed at the end of the RTOS startup sequence.
	//
	// PUT YOUR CODE HERE
	//

	return;
}

void UserHighFrequencyTaskInit(void)
{
	// User code to be executed ONE TIME the first time the high frequency task is run.
	//
	// PUT YOUR CODE HERE
	//
	return;
}

void UserMediumFrequencyTaskInit(void)
{
	// User code to be executed ONE TIME the first time the medium frequency task is run
	//
	// PUT YOUR CODE HERE
	//
	return;
}


void UserHighFrequencyTaskRun(void)
{
	// The default frequency at which this code runs is 200Hz.
	// This code runs after sensors are sampled.
	// In general, try to keep "high intensity" code out of UserHighFrequencyTaskRun.
	// The high frequency task also has highest priority.
	//
	// PUT YOUR CODE HERE
	//
	return;
}

void UserMediumFrequencyTaskRun(void)
{
	static int32 iThrottle = 0;

	// this function is called after the Kalman filter loop at a rate of SENSORFS / OVERSAMPLE_RATIO Hz.
	// with the default settings this is 200Hz/8=25Hz giving a smooth video quality display on the 
	// PC and Android user interfaces.
	// the UART (serial over USB and over Bluetooth) is limited to 115kbps which is more than adequate for
	// the 31kbps needed at the default 25Hz output rate but insufficient for 100Hz or 200Hz output rates.
	// since there is little point is providing output data faster than 25Hz video rates, this function
	// throttles the packet rate to a maximum of MAXPACKETRATE=25Hz.

	// check for any need to throttle the output rate
#define MAXPACKETRATE 25
#define RATERESOLUTION 1000
	if (((int32)MAXPACKETRATE * (int32)OVERSAMPLE_RATIO) >= (int32)SENSORFS)
	{
		// no UART bandwidth problem: transmit the packets over UART (USB and Bluetooth)
		CreateAndSendBluetoothPacketsViaUART(UART_DeviceData);
	}
	else
	{
		// throttle back by fractional multiplier (OVERSAMPLE_RATIO * MAXPACKETRATE) / SENSORFS
		// the increment applied to iThrottle is in the range 0 to (RATERESOLUTION - 1)
		iThrottle += ((int32)OVERSAMPLE_RATIO * (int32) MAXPACKETRATE * (int32)RATERESOLUTION) / SENSORFS;
		if (iThrottle >= RATERESOLUTION)
		{			
			// update the throttle counter and transmit the packets over UART (USB and Bluetooth)
			iThrottle -= RATERESOLUTION;
			CreateAndSendBluetoothPacketsViaUART(UART_DeviceData);
		}
	}

	//
	// PUT YOUR CODE HERE
	//
	
	return;
}
