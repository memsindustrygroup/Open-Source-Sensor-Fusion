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
// This is a Processor-Expert generated file which is then updated with custom
// code on an application-specific basic.  It contains various functions invoked
// from hardware interrupt handlers.  This also includes the UART input command
// interpreter.
//
#include "Cpu.h"
#include "Events.h"
#include "mqx_tasks.h"

/* User includes (#include below this line is not maintained by Processor Expert) */
#include "include_all.h"


// called on NMI
void Cpu_OnNMIINT(void)
{
	// return with no action
	return;
}

void FTM_OnCounterRestart(LDD_TUserData *UserDataPtr)
{
	// this function is called (typically at 200Hz) by the hardware clock interrupt and drives the sensor
	// read function and indirectly the fusion and magnetic calibration functions which
	// are software locked to the sensor read process

	// always enable the sensor read task
	// update time stamp counter
	mqxglobals.FTMTimestamp += (uint32)mqxglobals.FTMReload;
	// enable the (typically 200Hz) sensor read event FSL_SAMPLING_EVENT
	// with a mask of 1 (least significant bit set)
	_lwevent_set(&(mqxglobals.SamplingEventStruct), 1);

	return;
}


void UART_OnBlockSent(LDD_TUserData *UserDataPtr)
{
	// return with no action
	return;
}

void I2C_OnMasterBlockSent(LDD_TUserData *UserDataPtr)
{
	// set the I2C data sent flag
	mqxglobals.I2C_Status |= I2C_SENT_FLAG;

	return;
}


void I2C_OnMasterBlockReceived(LDD_TUserData *UserDataPtr)
{
	// set the I2C data received flag
	mqxglobals.I2C_Status |= I2C_RCVD_FLAG;

	return;
}

void I2C_OnError(LDD_TUserData *UserDataPtr)
{
	// return with no action
	return;
}

void UART_OnTxComplete(LDD_TUserData *UserDataPtr)
{
#ifdef DEEPSLEEP
	SCB_SCR |= SCB_SCR_SLEEPDEEP_MASK; // Enable full STOP mode
#endif
	return;
}

void UART_OnBlockReceived(LDD_TUserData *UserDataPtr)
{
	int32 isum;			// 32 bit identifier

	// this function is called when a complete UART block has been received which is either
	// 4 byte head command (physical sensors) or
	// 4 byte command plus 18 sensor data bytes (simulated sensors)

	// calculate a unique 32 bit identifier from the four command characters received
	isum = ((((((int32)sUARTInputBuf[0] << 8) + sUARTInputBuf[1]) << 8) + sUARTInputBuf[2]) << 8) + sUARTInputBuf[3];

	// execute the bluetooth command received
	switch (isum)
	{
	// "VG+ " = enable angular velocity packet transmission
	case ((((('V' << 8) + 'G') << 8) + '+') << 8) + ' ':
				globals.AngularVelocityPacketOn = true; 
	break;
	// "VG- " = disable angular velocity packet transmission
	case ((((('V' << 8) + 'G') << 8) + '-') << 8) + ' ':
				globals.AngularVelocityPacketOn = false; 
	break;
	// "DB+ " = enable debug packet transmission
	case ((((('D' << 8) + 'B') << 8) + '+') << 8) + ' ':
				globals.DebugPacketOn = true;
	break;
	// "DB- " = disable debug packet transmission
	case ((((('D' << 8) + 'B') << 8) + '-') << 8) + ' ':
				globals.DebugPacketOn = false; 
	break;
	// "Q3  " = transmit 3-axis accelerometer quaternion in standard packet
	case ((((('Q' << 8) + '3') << 8) + ' ') << 8) + ' ':
				globals.QuaternionPacketType = Q3;
	break;
	// "Q3M " = transmit 3-axis magnetometer quaternion in standard packet
	case ((((('Q' << 8) + '3') << 8) + 'M') << 8) + ' ':
				globals.QuaternionPacketType = Q3M;
	break;
	// "Q3G " = transmit 3-axis gyro quaternion in standard packet
	case ((((('Q' << 8) + '3') << 8) + 'G') << 8) + ' ':
				globals.QuaternionPacketType = Q3G; 
	break;
	// "Q6MA" = transmit 6-axis mag/accel quaternion in standard packet
	case ((((('Q' << 8) + '6') << 8) + 'M') << 8) + 'A':
				globals.QuaternionPacketType = Q6MA; 
	break;	
	// "Q6AG" = transmit 6-axis accel/gyro quaternion in standard packet
	case ((((('Q' << 8) + '6') << 8) + 'A') << 8) + 'G':
				globals.QuaternionPacketType = Q6AG;
	break;
	// "Q9  " = transmit 9-axis quaternion in standard packet (default)
	case ((((('Q' << 8) + '9') << 8) + ' ') << 8) + ' ':
				globals.QuaternionPacketType = Q9;
	break;
	// "RPC+" = Roll/Pitch/Compass on
	case ((((('R' << 8) + 'P') << 8) + 'C') << 8) + '+':
				globals.RPCPacketOn = true; 
	break;
	// "RPC-" = Roll/Pitch/Compass off
	case ((((('R' << 8) + 'P') << 8) + 'C') << 8) + '-':
				globals.RPCPacketOn = false; 
	break;
	// "ALT+" = Altitude packet on
	case ((((('A' << 8) + 'L') << 8) + 'T') << 8) + '+':
				globals.AltPacketOn = true; 
	break;
	// "ALT-" = Altitude packet off
	case ((((('A' << 8) + 'L') << 8) + 'T') << 8) + '-':
				globals.AltPacketOn = false; 
	break;
	
	// "RST " = Soft reset
	case ((((('R' << 8) + 'S') << 8) + 'T') << 8) + ' ':	
		Fusion_Init();
		mqxglobals.FTMTimestamp = 0;
	break;
	
	default:
		// no action
	break;
	}

	// tell UART to receive bluetooth packets and generate the next callback event to this same function
	// when the specified number of bytes have been received. this function is non-blocking
	// receive 4 bytes specifying the quaternion to be transmitted back
	UART_ReceiveBlock(UART_DeviceData, sUARTInputBuf, 4);

	return;
}
