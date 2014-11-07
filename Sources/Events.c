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
	// Disable NMI pin (some boards do not have pullups)
    SIM_SCGC5 |= (uint32_t)SIM_SCGC5_PORTA_MASK; /* NMI and PORTA clock gate enable */
    PORTA_PCR4 &= PORT_PCR_MUX_MASK;
    /* enable input with pull up enable not NMI */
    PORTA_PCR4 |= PORT_PCR_MUX(01) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
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
	int32 isum;			// 32 bit command identifier
	int16 nbytes;		// number of bytes received
	int16 i, j;			// loop counters

	// this function is called when one or more characters have arrived in the UART's receive buffer
	// incoming characters are placed in a delay line and processed whenever a valid command is received.
	// this provides resilience against loss of incoming characters.
	// note also that although this callback is theoretically called whenever a single byte is received, 
	// in practice there may be bursts of more than one byte in the receive buffer.
	// all received bytes are processed before this callback is executed.

	// determine how many bytes are available in the UART receive buffer
	nbytes = UART_GetReceivedDataNum(UART_DeviceData);

	// parse all received bytes in sUARTInputBuf into the iCommand delay line
	for (i = 0; i < nbytes; i++)
	{
		// shuffle the iCommand delay line and add the new command byte
		for (j = 0; j < 3; j++)
			iCommand[j] = iCommand[j + 1];
		iCommand[3] = sUARTInputBuf[i];
		
		// check if we have a valid command yet
		isum = ((((((int32)iCommand[0] << 8) + iCommand[1]) << 8) + iCommand[2]) << 8) + iCommand[3];
		switch (isum)
		{
		// "VG+ " = enable angular velocity packet transmission
		case ((((('V' << 8) + 'G') << 8) + '+') << 8) + ' ':
			globals.AngularVelocityPacketOn = true;
			iCommand[3] = '~';
		break;
		// "VG- " = disable angular velocity packet transmission
		case ((((('V' << 8) + 'G') << 8) + '-') << 8) + ' ':
			globals.AngularVelocityPacketOn = false; 
			iCommand[3] = '~';
		break;
		
		// "DB+ " = enable debug packet transmission
		case ((((('D' << 8) + 'B') << 8) + '+') << 8) + ' ':
			globals.DebugPacketOn = true;
			iCommand[3] = '~';
		break;
		// "DB- " = disable debug packet transmission
		case ((((('D' << 8) + 'B') << 8) + '-') << 8) + ' ':
			globals.DebugPacketOn = false; 
			iCommand[3] = '~';
		break;
		
		// "Q3  " = transmit 3-axis accelerometer quaternion in standard packet
		case ((((('Q' << 8) + '3') << 8) + ' ') << 8) + ' ':
	#if defined COMPUTE_3DOF_G_BASIC
			globals.QuaternionPacketType = Q3;
			iCommand[3] = '~';
	#endif
		break;
		// "Q3M " = transmit 3-axis magnetometer quaternion in standard packet
		case ((((('Q' << 8) + '3') << 8) + 'M') << 8) + ' ':
	#if defined COMPUTE_3DOF_B_BASIC
			globals.QuaternionPacketType = Q3M;
			iCommand[3] = '~';
	#endif
		break;
		// "Q3G " = transmit 3-axis gyro quaternion in standard packet
		case ((((('Q' << 8) + '3') << 8) + 'G') << 8) + ' ':
	#if defined COMPUTE_3DOF_Y_BASIC
			globals.QuaternionPacketType = Q3G; 
			iCommand[3] = '~';
	#endif
		break;
		// "Q6MA" = transmit 6-axis mag/accel quaternion in standard packet
		case ((((('Q' << 8) + '6') << 8) + 'M') << 8) + 'A':
	#if defined COMPUTE_6DOF_GB_BASIC
			globals.QuaternionPacketType = Q6MA;
			iCommand[3] = '~';
	#endif
		break;	
		// "Q6AG" = transmit 6-axis accel/gyro quaternion in standard packet
		case ((((('Q' << 8) + '6') << 8) + 'A') << 8) + 'G':
	#if defined COMPUTE_6DOF_GY_KALMAN
			globals.QuaternionPacketType = Q6AG;
			iCommand[3] = '~';
	#endif
		break;
		// "Q9  " = transmit 9-axis quaternion in standard packet (default)
		case ((((('Q' << 8) + '9') << 8) + ' ') << 8) + ' ':
	#if defined COMPUTE_9DOF_GBY_KALMAN
			globals.QuaternionPacketType = Q9;
			iCommand[3] = '~';
	#endif
		break;
		
		// "RPC+" = Roll/Pitch/Compass on
		case ((((('R' << 8) + 'P') << 8) + 'C') << 8) + '+':
			globals.RPCPacketOn = true; 
			iCommand[3] = '~';
		break;
		// "RPC-" = Roll/Pitch/Compass off
		case ((((('R' << 8) + 'P') << 8) + 'C') << 8) + '-':
			globals.RPCPacketOn = false; 
			iCommand[3] = '~';
		break;
		
		// "ALT+" = Altitude packet on
		case ((((('A' << 8) + 'L') << 8) + 'T') << 8) + '+':
			globals.AltPacketOn = true; 
			iCommand[3] = '~';
		break;
		// "ALT-" = Altitude packet off
		case ((((('A' << 8) + 'L') << 8) + 'T') << 8) + '-':
			globals.AltPacketOn = false; 
			iCommand[3] = '~';
		break;

		// "RST " = Soft reset
		case ((((('R' << 8) + 'S') << 8) + 'T') << 8) + ' ':	
			Fusion_Init();
			mqxglobals.FTMTimestamp = 0;
			iCommand[3] = '~';
		break;

		default:
			// no action
			break;
		}	
	} // end of loop over received characters

	// generate the next callback event to this function when the next character arrives
	// this function is non-blocking
	UART_ReceiveBlock(UART_DeviceData, sUARTInputBuf, 1);

	return;
}
