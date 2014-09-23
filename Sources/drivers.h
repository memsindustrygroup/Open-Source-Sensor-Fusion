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
#ifndef DRIVERS_H
#define DRIVERS_H

// compile time constants
#define I2C_BUF_LEN           16
#define UART_INPUT_BUFFER_SIZE 	4
#define UART_OUTPUT_BUFFER_SIZE 256
#define I2C_SENT_FLAG       0x01
#define I2C_RCVD_FLAG       0x02

// globals defined in FSL_utils.c
extern uint8 sUARTOutputBuf[UART_OUTPUT_BUFFER_SIZE];
extern uint8 sUARTInputBuf[UART_INPUT_BUFFER_SIZE];

// functions defined in FSL_utils.c
int8 MPL3115_Init(LDD_TDeviceData *DeviceDataPtr, struct PressureSensor *pthisPressure);
void FXOS8700_Init(LDD_TDeviceData *DeviceDataPtr, struct AccelSensor *pthisAccel, struct MagSensor *pthisMag);
void FXAS21000_Init(LDD_TDeviceData *DeviceDataPtr, struct GyroSensor *pthisGyro);
void FXAS21002_Init(LDD_TDeviceData *DeviceDataPtr, struct GyroSensor *pthisGyro);
void MMA8652_Init(LDD_TDeviceData *DeviceDataPtr, struct AccelSensor *pthisAccel);
void MAG3110_Init(LDD_TDeviceData *DeviceDataPtr, struct MagSensor *pthisMag);
void MPL3115_ReadData(LDD_TDeviceData *DeviceDataPtr, struct PressureSensor *pthisPressure);
void FXOS8700_ReadData(LDD_TDeviceData *DeviceDataPtr, struct AccelSensor *pthisAccel, struct MagSensor *pthisMag);
void FXAS21000_ReadData(LDD_TDeviceData *DeviceDataPtr, struct GyroSensor *pthisGyro, int16 irow);
void FXAS21002_ReadData(LDD_TDeviceData *DeviceDataPtr, struct GyroSensor *pthisGyro, int16 irow);
void MMA8652_ReadData(LDD_TDeviceData *DeviceDataPtr, struct AccelSensor *pthisAccel);
void MAG3110_ReadData(LDD_TDeviceData *DeviceDataPtr, struct MagSensor *pthisMag);
void sBufAppendItem(uint8* pDest, uint32* pIndex, uint8* pSource, uint16 iBytesToCopy);
void BlueRadios_Init(LDD_TDeviceData *DeviceDataPtr);
void CreateAndSendBluetoothPacketsViaUART(LDD_TDeviceData *DeviceDataPtr);

#endif // DRIVERS_H
