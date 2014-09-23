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
// This file contains sensor drivers as well as the function which create the fusion output
// packets via UART.
//
// Sensor drivers require logical device drivers (for I2C) created via Processor Expert.
// If you change sensors and/or MCU architecture, these will need to be replaced as appropriate.
//
#include "Events.h"
#include "mqx_tasks.h"
#include "I2C.h"
#include "UART.h"
#include "FTM.h"
#include "include_all.h"
#include "string.h"

// I2C and UART global buffers
uint8 I2C_Buf[I2C_BUF_LEN];
uint8 sUARTOutputBuf[UART_OUTPUT_BUFFER_SIZE];
uint8 sUARTInputBuf[UART_INPUT_BUFFER_SIZE];

// sensor physical I2C addresses
#define MPL3115_I2C_ADDR		0x60
#define FXOS8700_I2C_ADDR		0x1E
#define FXAS21000_I2C_ADDR		0x20
#define FXAS21002_I2C_ADDR		0x20
#define MMA8652_I2C_ADDR		0x1D
#define MAG3110_I2C_ADDR		0x0E

// MPL3115 registers and constants
#define MPL3115_STATUS					0x00
#define MPL3115_OUT_P_MSB				0x01
#define MPL3115_WHO_AM_I				0x0C
#define MPL3115_CTRL_REG1       		0x26
#define MPL3115_WHO_AM_I_VALUE			0xC4

// FXOS8700 registers and constants
#define FXOS8700_OUT_X_MSB       	  	0x01
#define FXOS8700_WHO_AM_I      			0x0D
#define FXOS8700_XYZ_DATA_CFG       	0x0E
#define FXOS8700_CTRL_REG1        	 	0x2A
#define FXOS8700_CTRL_REG2        	 	0x2B
#define FXOS8700_M_CTRL_REG1         	0x5B
#define FXOS8700_M_CTRL_REG2        	0x5C
#define FXOS8700_WHO_AM_I_VALUE     	0xC7

// FXAS21000 registers and constants
#define FXAS21000_DATA_REG            	0x01
#define FXAS21000_WHO_AM_I        		0x0C
#define FXAS21000_CTRL_REG0           	0x0D
#define FXAS21000_CTRL_REG1           	0x13
#define FXAS21000_WHO_AM_I_VALUE		0xD1

// FXAS21002 registers and constants
#define FXAS21002_DATA_REG            	0x01 
#define FXAS21002_WHO_AM_I        		0x0C 
#define FXAS21002_CTRL_REG0           	0x0D
#define FXAS21002_CTRL_REG1           	0x13
#define FXAS21002_WHO_AM_I_VALUE		0xD4

// MMA8652 registers and constants
#define MMA8652_STATUS					0x00
#define MMA8652_OUT_X_MSB       	  	0x01
#define MMA8652_WHO_AM_I        		0x0D
#define MMA8652_XYZ_DATA_CFG     	  	0x0E
#define MMA8652_CTRL_REG1           	0x2A
#define MMA8652_CTRL_REG2           	0x2B
#define MMA8652_WHO_AM_I_VALUE			0x4A

// MAG3110 registers and constants
#define MAG3110_STATUS					0x00
#define MAG3110_OUT_X_MSB       	  	0x01
#define MAG3110_WHO_AM_I      			0x07
#define MAG3110_CTRL_REG1        	 	0x10
#define MAG3110_CTRL_REG2         		0x11
#define MAG3110_WHO_AM_I_VALUE     		0xC4

// initialize MPL3115 pressure and temperature sensor
int8 MPL3115_Init(LDD_TDeviceData *DeviceDataPtr, struct PressureSensor *pthisPressure)
{
	LDD_I2C_TBusState BusState;		// I2C bus state
	LDD_I2C_TErrorMask  MPL3115_I2C_Error;

	// set up the MPL3115 I2C address
	I2C_SelectSlaveDevice(DeviceDataPtr, LDD_I2C_ADDRTYPE_7BITS, MPL3115_I2C_ADDR);

	// write 0000 0000 = 0x00 to MPL3115_CTRL_REG1 to place the MPL3115 in Standby
	// [7]: ALT=0
	// [6]: RAW=0 
	// [5-3]: OS=000 
	// [2]: RST=0
	// [1]: OST=0
	// [0]: SBYB=0 to enter standby
	I2C_Buf[0] = MPL3115_CTRL_REG1;		// byte 0 is the destination register
	I2C_Buf[1] = 0x00;						// byte 1 is the data to write
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;			// reset the I2C sent flag

	// transmit the bytes
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);

	// loop while no error detected and the I2C sent callback has not yet set the sent flag
	do
	{
		// read the error flag
		// ERR_OK = 0x00: device is present
		// ERR_DISABLED = 0x07: device is disabled
		// ERR_SPEED = 0x01: device does not work in the active speed mode
		I2C_GetError(DeviceDataPtr, &MPL3115_I2C_Error);
	}
	while ((!MPL3115_I2C_Error) && !(mqxglobals.I2C_Status & I2C_SENT_FLAG));

	// return immediately with error condition if MPL3115 is not present
	if (MPL3115_I2C_Error)
		return false;

	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// alternative for barometer mode
	// write 0011 1001 = 0x39 to configure MPL3115 and enter Active mode
	// [7]: ALT=0 for pressure measurements

	// write 1011 1001 = 0xB9 to configure MPL3115 and enter Active mode
	// [7]: ALT=1 for altitude measurements
	// [6]: RAW=0 to disable raw measurements
	// [5-3]: OS=111 for OS ratio=128 for maximum internal averaging with 512ms output interval
	// [2]: RST=0 do not enter reset
	// [1]: OST=0 do not initiate a reading
	// [0]: SBYB=1 to enter active mode
	I2C_Buf[0] = MPL3115_CTRL_REG1;
	I2C_Buf[1] = 0xB9;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// store the gain terms in the pressure structure
#define MPL3115_MPERCOUNT 0.0000152587890625F		// 1/65536 fixed range for MPL3115	
#define MPL3115_CPERCPOUNT 0.00390625F				// 1/256 fixed range for MPL3115
	pthisPressure->fmPerCount = MPL3115_MPERCOUNT;
	pthisPressure->fCPerCount = MPL3115_CPERCPOUNT;

	return (true);
}

// initialize FXOS8700 accelerometer plus magnetometer sensor
void FXOS8700_Init(LDD_TDeviceData *DeviceDataPtr, struct AccelSensor *pthisAccel, struct MagSensor *pthisMag)
{
	LDD_I2C_TBusState BusState;		// I2C bus state

	// set up the FXOS8700 I2C address
	I2C_SelectSlaveDevice(DeviceDataPtr, LDD_I2C_ADDRTYPE_7BITS, FXOS8700_I2C_ADDR);

	// write 0000 0000 = 0x00 to CTRL_REG1 to place FXOS8700 into standby
	// [7-1] = 0000 000
	// [0]: active=0
	I2C_Buf[0] = FXOS8700_CTRL_REG1;
	I2C_Buf[1] = 0x00;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// write 0001 1111 = 0x1F to M_CTRL_REG1
	// [7]: m_acal=0: auto calibration disabled
	// [6]: m_rst=0: one-shot magnetic reset disabled
	// [5]: m_ost=0: one-shot magnetic measurement disabled
	// [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce magnetometer noise
	// [1-0]: m_hms=11=3: select hybrid mode with accel and magnetometer active
	I2C_Buf[0] = FXOS8700_M_CTRL_REG1;
	I2C_Buf[1] = 0x1F;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// write 0010 0000 = 0x20 to magnetometer control register 2
	// [7]: reserved
	// [6]: reserved
	// [5]: hyb_autoinc_mode=1 to map the magnetometer registers to follow the accelerometer registers
	// [4]: m_maxmin_dis=0 to retain default min/max latching even though not used
	// [3]: m_maxmin_dis_ths=0
	// [2]: m_maxmin_rst=0
	// [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle
	I2C_Buf[0] = FXOS8700_M_CTRL_REG2;
	I2C_Buf[1] = 0x20;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// write 0000 0001= 0x01 to XYZ_DATA_CFG register
	// [7]: reserved
	// [6]: reserved
	// [5]: reserved
	// [4]: hpf_out=0
	// [3]: reserved
	// [2]: reserved
	// [1-0]: fs=01 for 4g mode: 2048 counts / g = 8192 counts / g after 2 bit left shift
	I2C_Buf[0] = FXOS8700_XYZ_DATA_CFG;
	I2C_Buf[1] = 0x01;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// write 0000 0010 = 0x02 to CTRL_REG2 to set MODS bits
	// [7]: st=0: self test disabled
	// [6]: rst=0: reset disabled
	// [5]: unused
	// [4-3]: smods=00
	// [2]: slpe=0: auto sleep disabled
	// [1-0]: mods=10 for high resolution (maximum over sampling)
	I2C_Buf[0] = FXOS8700_CTRL_REG2;
	I2C_Buf[1] = 0x02;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// write 0000 1101 = 0x0D to accelerometer control register 1
	// [7-6]: aslp_rate=00
	// [5-3]: dr=001=1 for 200Hz data rate (when in hybrid mode)
	// [2]: lnoise=1 for low noise mode (since we're in 4g mode)
	// [1]: f_read=0 for normal 16 bit reads
	// [0]: active=1 to take the part out of standby and enable sampling
	I2C_Buf[0] = FXOS8700_CTRL_REG1;
	I2C_Buf[1] = 0x0D;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// store the gain terms in the accelerometer and magnetometer sensor structures
#define FXOS8700_UTPERCOUNT 0.1F      				// fixed range for FXOS8700 magnetometer
#define FXOS8700_COUNTSPERUT 10.0F					// must be reciprocal of FUTPERCOUNT
#define FXOS8700_GPERCOUNT 0.0001220703125F			// equal to 1/8192
	pthisAccel->fgPerCount = FXOS8700_GPERCOUNT;
	pthisMag->fuTPerCount = FXOS8700_UTPERCOUNT;
	pthisMag->fCountsPeruT = FXOS8700_COUNTSPERUT;

	return;
}

// initialize FXAS21000 gyroscope sensor
void FXAS21000_Init(LDD_TDeviceData *DeviceDataPtr, struct GyroSensor *pthisGyro)
{
	LDD_I2C_TBusState BusState;		// I2C bus state

	// set up the FXAS21000 I2C address
	I2C_SelectSlaveDevice(DeviceDataPtr, LDD_I2C_ADDRTYPE_7BITS, FXAS21000_I2C_ADDR);

	// write 0000 0000 = 0x00 to CTRL_REG1 to place FXOS21000 in Standby
	// [7]: ZR_cond=0
	// [6]: RST=0
	// [5]: ST=0 self test disabled
	// [4-2]: DR[2-0]=000 for 200Hz ODR
	// [1-0]: Active=0, Ready=0 for Standby mode
	I2C_Buf[0] = FXAS21000_CTRL_REG1;
	I2C_Buf[1] = 0x00;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// write 0000 0000 = 0x00 to CTRL_REG0 to configure range and filters
	// [7-6]: unused=00
	// [5]: SPIW=0 4 wire SPI (irrelevant)
	// [4-3]: SEL[1-0]=00 for 10Hz HPF at 200Hz ODR
	// [2]: HPF_EN=0 disable HPF
	// [1-0]: FS[1-0]=00 for 1600dps
	I2C_Buf[0] = FXAS21000_CTRL_REG0;
	I2C_Buf[1] = 0x00;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// write 0000 0010 = 0x02 to CTRL_REG1 to configure 200Hz ODR and enter Active mode
	// [7]: ZR_cond=0
	// [6]: RST=0
	// [5]: ST=0 self test disabled
	// [4-2]: DR[2-0]=000 for 200Hz ODR
	// [1-0]: Active=1, Ready=0 for Active mode
	I2C_Buf[0] = FXAS21000_CTRL_REG1;
	I2C_Buf[1] = 0x02;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// place the gain in the gyro structure
	// 14 bit scaling is 0.2, 0.1, 0.05, 0.025dps/count for 1600, 800, 400, 200dps ranges
	// 14 bit scaling is 5, 10, 20, 40 counts/dps for 1600, 800, 400, 200dps ranges
	// 16 bit scaling is 0.05, 0.025, 0.0125, 0.00625dps/count for 1600, 800, 400, 200dps ranges
	// 16 bit scaling is 20, 40, 80, 160 counts/dps for 1600, 800, 400, 200dps ranges
	// our I2C driver reads 16 bit data left shifted two bits from FXAS21000
#define FXAS21000_COUNTSPERDEGPERSEC 20.0F     		// 1600dps range
#define FXAS21000_DEGPERSECPERCOUNT 0.05F			// must be reciprocal of FCOUNTSPERDEGPERSEC
	pthisGyro->fDegPerSecPerCount = FXAS21000_DEGPERSECPERCOUNT;

	return;
}

// initialize FXAS21002 gyroscope sensor
void FXAS21002_Init(LDD_TDeviceData *DeviceDataPtr, struct GyroSensor *pthisGyro)
{
	LDD_I2C_TBusState BusState;		// I2C bus state

	// set up the FXAS21002 I2C address
	I2C_SelectSlaveDevice(DeviceDataPtr, LDD_I2C_ADDRTYPE_7BITS, FXAS21002_I2C_ADDR);

	// write 0000 0000 = 0x00 to CTRL_REG1 to place FXOS21002 in Standby
	// [7]: ZR_cond=0
	// [6]: RST=0
	// [5]: ST=0 self test disabled
	// [4-2]: DR[2-0]=000 for 800Hz
	// [1-0]: Active=0, Ready=0 for Standby mode
	I2C_Buf[0] = FXAS21002_CTRL_REG1;
	I2C_Buf[1] = 0x00;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// write 0000 0000 = 0x00 to CTRL_REG0 to configure range and filters
	// [7-6]: BW[1-0]=00, LPF disabled
	// [5]: SPIW=0 4 wire SPI (irrelevant)
	// [4-3]: SEL[1-0]=00 for 10Hz HPF at 200Hz ODR
	// [2]: HPF_EN=0 disable HPF
	// [1-0]: FS[1-0]=00 for 1600dps (TBD CHANGE TO 2000dps when final trimmed parts available)
	I2C_Buf[0] = FXAS21002_CTRL_REG0;
	I2C_Buf[1] = 0x00;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// write 0000 1010 = 0x0A to CTRL_REG1 to configure 200Hz ODR and enter Active mode
	// [7]: ZR_cond=0
	// [6]: RST=0
	// [5]: ST=0 self test disabled
	// [4-2]: DR[2-0]=010 for 200Hz ODR
	// [1-0]: Active=1, Ready=0 for Active mode
	I2C_Buf[0] = FXAS21002_CTRL_REG1;
	I2C_Buf[1] = 0x0A;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// place the gain in the gyro structure
	#define FXAS21002_COUNTSPERDEGPERSEC 16.0F     		// for production 2000dps range (2000dps=32000)
	#define FXAS21002_DEGPERSECPERCOUNT 0.0625F			// must be reciprocal of FCOUNTSPERDEGPERSEC
	pthisGyro->fDegPerSecPerCount = FXAS21002_DEGPERSECPERCOUNT;

	return;
}

// initialize MMA8652 accelerometer sensor
void MMA8652_Init(LDD_TDeviceData *DeviceDataPtr, struct AccelSensor *pthisAccel)
{
	LDD_I2C_TBusState BusState;		// I2C bus state

	// set up the MMA8652 I2C address
	I2C_SelectSlaveDevice(DeviceDataPtr, LDD_I2C_ADDRTYPE_7BITS, MMA8652_I2C_ADDR);

	// write 0000 0000 = 0x00 to CTRL_REG1 to place MMA8652 into standby
	// [7-1] = 0000 000
	// [0]: active=0
	I2C_Buf[0] = MMA8652_CTRL_REG1;
	I2C_Buf[1] = 0x00;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// write 0000 0001 = 0x01 to XYZ_DATA_CFG register to set g range
	// [7-5]: reserved=000
	// [4]: HPF_OUT=0
	// [3-2]: reserved=00
	// [1-0]: FS=01 for +/-4g: 512 counts / g = 8192 counts / g after 4 bit left shift
	I2C_Buf[0] = MMA8652_XYZ_DATA_CFG;
	I2C_Buf[1] = 0x01;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// write 0000 0010 = 0x02 to CTRL_REG2 to set MODS bits
	// [7]: ST=0: self test disabled
	// [6]: RST=0: reset disabled
	// [5]: unused
	// [4-3]: SMODS=00
	// [2]: SLPE=0: auto sleep disabled
	// [1-0]: mods=10 for high resolution (maximum over sampling)
	I2C_Buf[0] = MMA8652_CTRL_REG2;
	I2C_Buf[1] = 0x02;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// write 0001 0001 = 0x11 to CTRL_REG1
	// [7-6]: aslp_rate=00
	// [5-3]: dr=010 for 200Hz data rate
	// [2]: unused=0
	// [1]: f_read=0 for normal 16 bit reads
	// [0]: active=1 to take the part out of standby and enable sampling
	I2C_Buf[0] = MMA8652_CTRL_REG1;
	I2C_Buf[1] = 0x11;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// store the gain terms in the accelerometer and magnetometer sensor structures
#define MMA8652_GPERCOUNT 0.0001220703125F			// equal to 1/8192
	pthisAccel->fgPerCount = MMA8652_GPERCOUNT;

	return;
}

// initialize MAG3110 magnetometer sensor
void MAG3110_Init(LDD_TDeviceData *DeviceDataPtr, struct MagSensor *pthisMag)
{
	LDD_I2C_TBusState BusState;		// I2C bus state

	// set up the MAG3110 I2C address
	I2C_SelectSlaveDevice(DeviceDataPtr, LDD_I2C_ADDRTYPE_7BITS, MAG3110_I2C_ADDR);

	// write 0000 0000 = 0x00 to CTRL_REG1 to place MMAG3110 into standby
	// [7-1] = 0000 000
	// [0]: AC=0 for standby
	I2C_Buf[0] = MAG3110_CTRL_REG1;
	I2C_Buf[1] = 0x00;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// write 1001 0000 = 0x90 to CTRL_REG2
	// [7]: AUTO_MRST_EN = 1: enable degaussing
	// [6]: unused=0
	// [5]: RAW=0: normal mode
	// [4]: Mag_RST=1: enable a single degauss
	// [3-0]: unused=0
	I2C_Buf[0] = MAG3110_CTRL_REG2;
	I2C_Buf[1] = 0x90;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// write 0000 0001 = 0x01 to CTRL_REG1 to place MMAG3110 into standby
	// [7-5]: DR=000 for 80Hz ODR
	// [4-3]: OS=00 for no oversampling
	// [2]: FT=0 for normal reads
	// [1]: TM=0 to not trigger immediate measurement
	// [0]: AC=1 for active mode
	I2C_Buf[0] = MAG3110_CTRL_REG1;
	I2C_Buf[1] = 0x01;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 2, LDD_I2C_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// store the gain terms in the magnetometer sensor structure
#define MAG3110_UTPERCOUNT 0.1F      				// fixed range for MAG3110 magnetometer
#define MAG3110_COUNTSPERUT 10.0F					// must be reciprocal of FUTPERCOUNT
	pthisMag->fuTPerCount = MAG3110_UTPERCOUNT;
	pthisMag->fCountsPeruT = MAG3110_COUNTSPERUT;

	return;
}

// read MPL3115 pressure sensor over I2C
void MPL3115_ReadData(LDD_TDeviceData *DeviceDataPtr, struct PressureSensor *pthisPressure)
{
	LDD_I2C_TBusState BusState;		// I2C bus state

	// set up the MPL3115 I2C address
	I2C_SelectSlaveDevice(DeviceDataPtr, LDD_I2C_ADDRTYPE_7BITS, MPL3115_I2C_ADDR);

	// set up the address of the first data register
	I2C_Buf[0] = MPL3115_OUT_P_MSB;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 1, LDD_I2C_NO_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);

	// read the 5 bytes of sequential sensor data
	mqxglobals.I2C_Status &= ~I2C_RCVD_FLAG;
	I2C_MasterReceiveBlock(DeviceDataPtr, I2C_Buf, 5, LDD_I2C_SEND_STOP);
	// wait until the I2C received callback function sets the received flag
	while ((mqxglobals.I2C_Status & I2C_RCVD_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// place the read buffer into the 32 bit altitude and 16 bit temperature
	pthisPressure->iHpFast = (I2C_Buf[0] << 24) | (I2C_Buf[1] << 16) | (I2C_Buf[2] << 8);
	pthisPressure->iTpFast = (I2C_Buf[3] << 8) | I2C_Buf[4];

	// use this line if the MPL3115 is to be used in pressure mode
	//pthisPressure->iPpFast = (I2C_Buf[0] << 16) | (I2C_Buf[1] << 8) | (I2C_Buf[2] << 0);

	return;
}

// read FXOS8700 accelerometer and magnetometer data over I2C
void FXOS8700_ReadData(LDD_TDeviceData *DeviceDataPtr, struct AccelSensor *pthisAccel, struct MagSensor *pthisMag)
{
	LDD_I2C_TBusState BusState;		// I2C bus state

	// set up the FXOS8700 I2C address
	I2C_SelectSlaveDevice(DeviceDataPtr, LDD_I2C_ADDRTYPE_7BITS, FXOS8700_I2C_ADDR);
	// set up the address of the first output register
	I2C_Buf[0] = FXOS8700_OUT_X_MSB;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 1, LDD_I2C_NO_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);

	// read the 12 bytes of sequential sensor data
	mqxglobals.I2C_Status &= ~I2C_RCVD_FLAG;
	I2C_MasterReceiveBlock(DeviceDataPtr, I2C_Buf, 12, LDD_I2C_SEND_STOP);
	// wait until the I2C received callback function sets the received flag
	while ((mqxglobals.I2C_Status & I2C_RCVD_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// place the 12 bytes read into the 16 bit accelerometer and magnetometer structures
	pthisAccel->iGpFast[X] = (I2C_Buf[0] << 8) | I2C_Buf[1];
	pthisAccel->iGpFast[Y] = (I2C_Buf[2] << 8) | I2C_Buf[3];
	pthisAccel->iGpFast[Z] = (I2C_Buf[4] << 8) | I2C_Buf[5];
	pthisMag->iBpFast[X] = (I2C_Buf[6] << 8) | I2C_Buf[7];
	pthisMag->iBpFast[Y] = (I2C_Buf[8] << 8) | I2C_Buf[9];
	pthisMag->iBpFast[Z] = (I2C_Buf[10] << 8) | I2C_Buf[11];

	// finally check for -32768 in the accelerometer and magnetometer data since
	// this value cannot be negated in a later HAL operation
	if (pthisAccel->iGpFast[X] == -32768) pthisAccel->iGpFast[X]++;
	if (pthisAccel->iGpFast[Y] == -32768) pthisAccel->iGpFast[Y]++;
	if (pthisAccel->iGpFast[Z] == -32768) pthisAccel->iGpFast[Z]++;
	if (pthisMag->iBpFast[X] == -32768) pthisMag->iBpFast[X]++;
	if (pthisMag->iBpFast[Y] == -32768) pthisMag->iBpFast[Y]++;
	if (pthisMag->iBpFast[Z] == -32768) pthisMag->iBpFast[Z]++;

	return;
}

// read FXAS21000 gyro data over I2C
void FXAS21000_ReadData(LDD_TDeviceData *DeviceDataPtr, struct GyroSensor *pthisGyro, int16 irow)
{
	LDD_I2C_TBusState BusState;		// I2C bus state

	// set up the FXAS21000 I2C address
	I2C_SelectSlaveDevice(DeviceDataPtr, LDD_I2C_ADDRTYPE_7BITS, FXAS21000_I2C_ADDR);

	// set up the address of the first output register
	I2C_Buf[0] = FXAS21000_DATA_REG;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 1, LDD_I2C_NO_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);

	// read the 6 bytes of sequential sensor data
	mqxglobals.I2C_Status &= ~I2C_RCVD_FLAG;
	I2C_MasterReceiveBlock(DeviceDataPtr, I2C_Buf, 6, LDD_I2C_SEND_STOP);
	// wait until the I2C received callback function sets the received flag
	while ((mqxglobals.I2C_Status & I2C_RCVD_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// place the read buffer into the 16 bit gyro structure
	pthisGyro->iYpFast[irow][X] = (I2C_Buf[0] << 8) | I2C_Buf[1];
	pthisGyro->iYpFast[irow][Y] = (I2C_Buf[2] << 8) | I2C_Buf[3];
	pthisGyro->iYpFast[irow][Z] = (I2C_Buf[4] << 8) | I2C_Buf[5];

	// finally check for -32768 which cannot be negated in a later HAL operation
	// this value can be obtained during clipping in rapid rotation
	if (pthisGyro->iYpFast[irow][X] == -32768) pthisGyro->iYpFast[irow][X]++;
	if (pthisGyro->iYpFast[irow][Y] == -32768) pthisGyro->iYpFast[irow][Y]++;
	if (pthisGyro->iYpFast[irow][Z] == -32768) pthisGyro->iYpFast[irow][Z]++;

	return;
}

// read FXAS21002 gyro data over I2C
void FXAS21002_ReadData(LDD_TDeviceData *DeviceDataPtr, struct GyroSensor *pthisGyro, int16 irow)
{
	LDD_I2C_TBusState BusState;		// I2C bus state

	// set up the FXAS21002 I2C address
	I2C_SelectSlaveDevice(DeviceDataPtr, LDD_I2C_ADDRTYPE_7BITS, FXAS21002_I2C_ADDR);

	// set up the address of the first output register
	I2C_Buf[0] = FXAS21002_DATA_REG;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 1, LDD_I2C_NO_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);

	// read the 6 bytes of sequential sensor data
	mqxglobals.I2C_Status &= ~I2C_RCVD_FLAG;
	I2C_MasterReceiveBlock(DeviceDataPtr, I2C_Buf, 6, LDD_I2C_SEND_STOP);
	// wait until the I2C received callback function sets the received flag
	while ((mqxglobals.I2C_Status & I2C_RCVD_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// place the read buffer into the 16 bit gyro structure
	pthisGyro->iYpFast[irow][X] = (I2C_Buf[0] << 8) | I2C_Buf[1];
	pthisGyro->iYpFast[irow][Y] = (I2C_Buf[2] << 8) | I2C_Buf[3];
	pthisGyro->iYpFast[irow][Z] = (I2C_Buf[4] << 8) | I2C_Buf[5];

	// finally check for -32768 which cannot be negated in a later HAL operation
	// this value can be obtained during clipping in rapid rotation
	if (pthisGyro->iYpFast[irow][X] == -32768) pthisGyro->iYpFast[irow][X]++;
	if (pthisGyro->iYpFast[irow][Y] == -32768) pthisGyro->iYpFast[irow][Y]++;
	if (pthisGyro->iYpFast[irow][Z] == -32768) pthisGyro->iYpFast[irow][Z]++;

	return;
}

// read MMA8652 accelerometer and magnetometer data over I2C
void MMA8652_ReadData(LDD_TDeviceData *DeviceDataPtr, struct AccelSensor *pthisAccel)
{
	LDD_I2C_TBusState BusState;		// I2C bus state

	// set up the MMA8652 I2C address
	I2C_SelectSlaveDevice(DeviceDataPtr, LDD_I2C_ADDRTYPE_7BITS, MMA8652_I2C_ADDR);
	// set up the address of the first output register
	I2C_Buf[0] = MMA8652_OUT_X_MSB;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 1, LDD_I2C_NO_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);

	// read the 6 bytes of sequential sensor data
	mqxglobals.I2C_Status &= ~I2C_RCVD_FLAG;
	I2C_MasterReceiveBlock(DeviceDataPtr, I2C_Buf, 6, LDD_I2C_SEND_STOP);
	// wait until the I2C received callback function sets the received flag
	while ((mqxglobals.I2C_Status & I2C_RCVD_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// place the 12 bytes read into the 16 bit accelerometer structure
	pthisAccel->iGpFast[X] = (I2C_Buf[0] << 8) | I2C_Buf[1];
	pthisAccel->iGpFast[Y] = (I2C_Buf[2] << 8) | I2C_Buf[3];
	pthisAccel->iGpFast[Z] = (I2C_Buf[4] << 8) | I2C_Buf[5];

	// finally check for -32768 in the accelerometer since
	// this value cannot be negated in a later HAL operation
	if (pthisAccel->iGpFast[X] == -32768) pthisAccel->iGpFast[X]++;
	if (pthisAccel->iGpFast[Y] == -32768) pthisAccel->iGpFast[Y]++;
	if (pthisAccel->iGpFast[Z] == -32768) pthisAccel->iGpFast[Z]++;

	return;
}

// read MAG3110 accelerometer and magnetometer data over I2C
void MAG3110_ReadData(LDD_TDeviceData *DeviceDataPtr, struct MagSensor *pthisMag)
{
	LDD_I2C_TBusState BusState;		// I2C bus state

	// set up the MAG3110 I2C address
	I2C_SelectSlaveDevice(DeviceDataPtr, LDD_I2C_ADDRTYPE_7BITS, MAG3110_I2C_ADDR);
	// set up the address of the first output register
	I2C_Buf[0] = MAG3110_OUT_X_MSB;
	mqxglobals.I2C_Status &= ~I2C_SENT_FLAG;
	I2C_MasterSendBlock(DeviceDataPtr, I2C_Buf, 1, LDD_I2C_NO_SEND_STOP);
	// wait until the I2C sent callback function sets the sent flag
	while ((mqxglobals.I2C_Status & I2C_SENT_FLAG) == 0);

	// read the 6 bytes of sequential sensor data
	mqxglobals.I2C_Status &= ~I2C_RCVD_FLAG;
	I2C_MasterReceiveBlock(DeviceDataPtr, I2C_Buf, 6, LDD_I2C_SEND_STOP);
	// wait until the I2C received callback function sets the received flag
	while ((mqxglobals.I2C_Status & I2C_RCVD_FLAG) == 0);
	// wait until the I2C bus is idle
	do
	{
		I2C_CheckBus(DeviceDataPtr, &BusState);
	} while (BusState != LDD_I2C_IDLE);

	// place the 12 bytes read into the 16 bit magnetometer structure
	pthisMag->iBpFast[X] = (I2C_Buf[0] << 8) | I2C_Buf[1];
	pthisMag->iBpFast[Y] = (I2C_Buf[2] << 8) | I2C_Buf[3];
	pthisMag->iBpFast[Z] = (I2C_Buf[4] << 8) | I2C_Buf[5];

	// finally check for -32768 in the magnetometer data since
	// this value cannot be negated in a later HAL operation
	if (pthisMag->iBpFast[X] == -32768) pthisMag->iBpFast[X]++;
	if (pthisMag->iBpFast[Y] == -32768) pthisMag->iBpFast[Y]++;
	if (pthisMag->iBpFast[Z] == -32768) pthisMag->iBpFast[Z]++;

	return;
}

// initialize BlueRadios BR-LE4.0-D2A Bluetooth module
void BlueRadios_Init(LDD_TDeviceData *DeviceDataPtr)
{
	uint16 ilen;		// command string length

	// transmit "ATSRM,2,0\r" to minimize traffic from the module
	// command "ATSRM": sets the module response mode which configures how verbose the module will be
	// 2: response mode at to minimal
	// 0: disconnected mode is command mode
	// \r: carriage return escape sequence
	strcpy((char *)sUARTOutputBuf, "ATSRM,2,0\r");
	ilen = strlen((char *)sUARTOutputBuf);
	UART_SendBlock(DeviceDataPtr, sUARTOutputBuf, ilen);

	// wait until all characters are transmitted
	while (UART_GetSentDataNum(DeviceDataPtr) != ilen)
		;

	// clear the input UART buffer of any characters returned by the Bluetooth module
	UART_ReceiveBlock(UART_DeviceData, sUARTInputBuf, UART_INPUT_BUFFER_SIZE);

	return;
}

// function appends a variable number of source bytes to a destimation buffer
// for transmission as the bluetooth packet
// bluetooth packets are delimited by inserting the special byte 0x7E at the start
// and end of packets. this function must therefore handle the case of 0x7E appearing
// as general data. this is done here with the substitutions:
// a) replace 0x7E by 0x7D and 0x5E (one byte becomes two) 
// b) replace 0x7D by 0x7D and 0x5D (one byte becomes two)
// the inverse mapping must be performed at the application receiving the bluetooth stream: ie:
// replace 0x7D and 0x5E with 0x7E
// replace 0x7D and 0x5D with 0x7D
// NOTE: do not use this function to append the start and end bytes 0x7E to the bluetooth
// buffer. instead add the start and end bytes 0x7E explicitly as in:
// sUARTOutputBuf[iByteCount++] = 0x7E;
void sBufAppendItem(uint8* pDest, uint32* pIndex, uint8* pSource, uint16 iBytesToCopy)
{
	uint16 i;			// loop counter

	// loop over number of bytes to add to the destination buffer
	for (i = 0; i < iBytesToCopy; i++)
	{
		// check for special case 1: replace 0x7E (start and end byte) with 0x7D and 0x5E
		if (pSource[i] == 0x7E)
		{
			pDest[(*pIndex)++] = 0x7D;
			pDest[(*pIndex)++] = 0x5E;
		}
		// check for special case 2: replace 0x7D with 0x7D and 0x5D
		else if (pSource[i] == 0x7D)
		{
			pDest[(*pIndex)++] = 0x7D;
			pDest[(*pIndex)++] = 0x5D;
		}
		else
			// general case, simply add this byte without change
		{
			pDest[(*pIndex)++] = pSource[i];
		}
	}

	return;
}

// set bluetooth packets out over UART
void CreateAndSendBluetoothPacketsViaUART(LDD_TDeviceData *DeviceDataPtr)
{
	struct fquaternion fq;		// quaternion to be transmitted
	float ftmp;
	uint32 iIndex;				// output buffer counter
	int32 tmpint32;				// scratch int32
	int16 tmpint16;				// scratch int16
	int16 iPhi, iThe, iRho;		// integer angles to be transmitted
	int16 iDelta;				// magnetic inclination angle if available
	int16 iOmega[3];			// scaled angular velocity vector
	uint16 isystick;			// algorithm systick time
	uint8 tmpuint8;				// scratch uint8
	uint8 flags;				// byte of flags
	int16 i, j, k;				// general purpose

#ifdef UART_OFF
	SCB_SCR |= SCB_SCR_SLEEPDEEP_MASK; // Enable full STOP mode
	return;  // SERIAL COMM IS NOT RUNNING
#else
	//    LED_RED_ClrVal(NULL);
	SCB_SCR &= (~SCB_SCR_SLEEPDEEP_MASK); // Disable full STOP mode
#endif

	// zero the counter for bytes accumulated into the transmit buffer
	iIndex = 0;

	// ***************************************************************
	// Main type 1: range 0 to 35 = 36 bytes
	// Debug type 2: range 0 to 7 = 8 bytes
	// Angular velocity type 3: range 0 to 13 = 14 bytes
	// Euler angles type 4: range 0 to 13 = 14 bytes
	// Altitude/Temp type 5: range 0 to 13 = 14 bytes
	// Magnetic type 6: range 0 to 15 = 16 bytes
	// Kalman packet 7: range 0 to 21 = 22 bytes
	// Total is 124 bytes vs 256 bytes in UART_OUTPUT_BUFFER_SIZE
	// at 25Hz, data rate is 25*124 = 3100 bytes/sec = 31.0kbaud
	// the UART is set to 115kbaud so about 27% is being used
	// ***************************************************************
	
	// ************************************************************************
	// fixed length packet type 1
	// this packet type is always transmitted
	// total size is 0 to 35 equals 36 bytes 
	// ************************************************************************

	// [0]: packet start byte (need a iIndex++ here since not using sBufAppendItem)
	sUARTOutputBuf[iIndex++] = 0x7E;

	// [1]: packet type 1 byte (iIndex is automatically updated in sBufAppendItem)
	tmpuint8 = 0x01;
	sBufAppendItem(sUARTOutputBuf, &iIndex, &tmpuint8, 1);

	// [2]: packet number byte
	sBufAppendItem(sUARTOutputBuf, &iIndex, &(globals.iPacketNumber), 1);
	globals.iPacketNumber++;

	// [6-3]: 1MHz time stamp (4 bytes)
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&(mqxglobals.FTMTimestamp), 4);

	// [12-7]: integer uncalibrated accelerometer data words
#if defined USE_FXOS8700 || defined USE_MMA8652
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&thisAccel.iGp[X], 2);
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&thisAccel.iGp[Y], 2);
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&thisAccel.iGp[Z], 2);
#else
	tmpint16 = 0;
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);	
#endif

	// [18-13]: integer calibrated magnetometer data words
#if defined USE_FXOS8700 || defined USE_MAG3110
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&thisMag.iBc[X], 2);
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&thisMag.iBc[Y], 2);
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&thisMag.iBc[Z], 2);
#else
	tmpint16 = 0;
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);	
#endif
	// [24-19]: uncalibrated gyro data words
#if defined USE_FXAS21000 || defined USE_FXAS21002
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&thisGyro.iYp[X], 2);
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&thisGyro.iYp[Y], 2);
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&thisGyro.iYp[Z], 2);
#else
	tmpint16 = 0;
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);	
#endif

	// initialize default quaternion, flags byte, angular velocity and orientation
	fq.q0 = 1.0F;
	fq.q1 = fq.q2 = fq.q3 = 0.0F;
	flags = 0x00;
	iOmega[X] = iOmega[Y] = iOmega[Z] = 0;
	iPhi = iThe = iRho = iDelta = 0;
	isystick = 0;

	// flags byte 33: quaternion type in least significant nibble
	// Q3:   coordinate nibble, 1
	// Q3M:	 coordinate nibble, 6
	// Q3G:	 coordinate nibble, 3
	// Q6MA: coordinate nibble, 2
	// Q6AG: coordinate nibble, 4
	// Q9:   coordinate nibble, 8

	// flags byte 33: coordinate in most significant nibble
	// Aerospace/NED:	0, quaternion nibble    
	// Android:	  		1, quaternion nibble
	// Windows 8: 		2, quaternion nibble

	// set the quaternion, flags, angular velocity and Euler angles
	switch (globals.QuaternionPacketType)
	{
	case Q3:
#ifdef COMPUTE_3DOF_G_BASIC
		fq = thisSV_3DOF_G_BASIC.fLPq;
		flags |= 0x01;
		iOmega[X] = (int16)(thisSV_3DOF_G_BASIC.fOmega[X] * 20.0F);
		iOmega[Y] = (int16)(thisSV_3DOF_G_BASIC.fOmega[Y] * 20.0F);
		iOmega[Z] = (int16)(thisSV_3DOF_G_BASIC.fOmega[Z] * 20.0F);
		iPhi = (int16) (10.0F * thisSV_3DOF_G_BASIC.fLPPhi);
		iThe = (int16) (10.0F * thisSV_3DOF_G_BASIC.fLPThe);
		iRho = (int16) (10.0F * thisSV_3DOF_G_BASIC.fLPRho);
		iDelta = 0;
		isystick = (uint16)(thisSV_3DOF_G_BASIC.systick / 20);
#endif
		break;
	case Q3M:
#ifdef COMPUTE_3DOF_B_BASIC
		fq = thisSV_3DOF_B_BASIC.fLPq;
		//	flags |= 0x02;
		flags |= 0x06;
		iOmega[X] = (int16)(thisSV_3DOF_B_BASIC.fOmega[X] * 20.0F);
		iOmega[Y] = (int16)(thisSV_3DOF_B_BASIC.fOmega[Y] * 20.0F);
		iOmega[Z] = (int16)(thisSV_3DOF_B_BASIC.fOmega[Z] * 20.0F);
		iPhi = (int16) (10.0F * thisSV_3DOF_B_BASIC.fLPPhi);
		iThe = (int16) (10.0F * thisSV_3DOF_B_BASIC.fLPThe);
		iRho = (int16) (10.0F * thisSV_3DOF_B_BASIC.fLPRho);
		iDelta = 0;
		isystick = (uint16)(thisSV_3DOF_B_BASIC.systick / 20);
#endif
		break;
	case Q3G:
#ifdef COMPUTE_3DOF_Y_BASIC
		fq = thisSV_3DOF_Y_BASIC.fq;
		//flags |= 0x01;
		flags |= 0x03;
		iOmega[X] = (int16)(thisSV_3DOF_Y_BASIC.fOmega[X] * 20.0F);
		iOmega[Y] = (int16)(thisSV_3DOF_Y_BASIC.fOmega[Y] * 20.0F);
		iOmega[Z] = (int16)(thisSV_3DOF_Y_BASIC.fOmega[Z] * 20.0F);
		iPhi = (int16) (10.0F * thisSV_3DOF_Y_BASIC.fPhi);
		iThe = (int16) (10.0F * thisSV_3DOF_Y_BASIC.fThe);
		iRho = (int16) (10.0F * thisSV_3DOF_Y_BASIC.fRho);
		iDelta = 0;
		isystick = (uint16)(thisSV_3DOF_Y_BASIC.systick / 20);
#endif
		break;
	case Q6MA:
#ifdef COMPUTE_6DOF_GB_BASIC
		fq = thisSV_6DOF_GB_BASIC.fLPq;
		flags |= 0x02;
		iOmega[X] = (int16)(thisSV_6DOF_GB_BASIC.fOmega[X] * 20.0F);
		iOmega[Y] = (int16)(thisSV_6DOF_GB_BASIC.fOmega[Y] * 20.0F);
		iOmega[Z] = (int16)(thisSV_6DOF_GB_BASIC.fOmega[Z] * 20.0F);
		iPhi = (int16) (10.0F * thisSV_6DOF_GB_BASIC.fLPPhi);
		iThe = (int16) (10.0F * thisSV_6DOF_GB_BASIC.fLPThe);
		iRho = (int16) (10.0F * thisSV_6DOF_GB_BASIC.fLPRho);
		iDelta = (int16) (10.0F * thisSV_6DOF_GB_BASIC.fLPDelta);
		isystick = (uint16)(thisSV_6DOF_GB_BASIC.systick / 20);
#endif
		break;
	case Q6AG:	
#ifdef COMPUTE_6DOF_GY_KALMAN
		fq = thisSV_6DOF_GY_KALMAN.fqPl;
		flags |= 0x04;
		iOmega[X] = (int16)(thisSV_6DOF_GY_KALMAN.fOmega[X] * 20.0F);
		iOmega[Y] = (int16)(thisSV_6DOF_GY_KALMAN.fOmega[Y] * 20.0F);
		iOmega[Z] = (int16)(thisSV_6DOF_GY_KALMAN.fOmega[Z] * 20.0F);
		iPhi = (int16) (10.0F * thisSV_6DOF_GY_KALMAN.fPhiPl);
		iThe = (int16) (10.0F * thisSV_6DOF_GY_KALMAN.fThePl);
		iRho = (int16) (10.0F * thisSV_6DOF_GY_KALMAN.fRhoPl);
		iDelta = 0;
		isystick = (uint16)(thisSV_6DOF_GY_KALMAN.systick / 20);
#endif
		break;
	case Q9:
#ifdef COMPUTE_9DOF_GBY_KALMAN
		fq = thisSV_9DOF_GBY_KALMAN.fqPl;
		flags |= 0x08;
		iOmega[X] = (int16)(thisSV_9DOF_GBY_KALMAN.fOmega[X] * 20.0F);
		iOmega[Y] = (int16)(thisSV_9DOF_GBY_KALMAN.fOmega[Y] * 20.0F);
		iOmega[Z] = (int16)(thisSV_9DOF_GBY_KALMAN.fOmega[Z] * 20.0F);
		iPhi = (int16) (10.0F * thisSV_9DOF_GBY_KALMAN.fPhiPl);
		iThe = (int16) (10.0F * thisSV_9DOF_GBY_KALMAN.fThePl);
		iRho = (int16) (10.0F * thisSV_9DOF_GBY_KALMAN.fRhoPl);
		iDelta = (int16) (10.0F * thisSV_9DOF_GBY_KALMAN.fDeltaPl);
		isystick = (uint16)(thisSV_9DOF_GBY_KALMAN.systick / 20);
#endif
		break;
	default:
		// use the default data already initialized
		break;
	}

	// [32-25]: scale the quaternion (30K = 1.0F) and add to the buffer
	tmpint16 = (int16)(fq.q0 * 30000.0F);
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
	tmpint16 = (int16)(fq.q1 * 30000.0F);
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
	tmpint16 = (int16)(fq.q2 * 30000.0F);
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
	tmpint16 = (int16)(fq.q3 * 30000.0F);
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);

	// set the coordinate system bits in flags from default NED (00)
#if (THISCOORDSYSTEM == ANDROID)
	// set the Android flag bits
	flags |= 0x10;
#elif  (THISCOORDSYSTEM == WIN8)
	// set the Win8 flag bits
	flags |= 0x20;
#endif // THISCOORDSYSTEM

	// [33]: add the flags byte to the buffer
	sBufAppendItem(sUARTOutputBuf, &iIndex, &flags, 1);

	// [34]: add the board ID byte
	tmpuint8 = THIS_BOARD_ID;
	sBufAppendItem(sUARTOutputBuf, &iIndex, &tmpuint8, 1);

	// [35]: add the tail byte for the standard packet type 1
	sUARTOutputBuf[iIndex++] = 0x7E;

	// ************************************************************************
	// Variable length debug packet type 2
	// currently total size is 0 to 7 equals 8 bytes 
	// ************************************************************************

	if (globals.DebugPacketOn)
	{
		// [0]: packet start byte
		sUARTOutputBuf[iIndex++] = 0x7E;

		// [1]: packet type 2 byte
		tmpuint8 = 0x02;
		sBufAppendItem(sUARTOutputBuf, &iIndex, &tmpuint8, 1);

		// [2]: packet number byte
		sBufAppendItem(sUARTOutputBuf, &iIndex, &(globals.iPacketNumber), 1);
		globals.iPacketNumber++;

		// [4-3] software version number
		tmpint16 = THISBUILD;
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
		
		// [6-5] systick count / 20
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&isystick, 2);

		// [7 in practice but can be variable]: add the tail byte for the debug packet type 2
		sUARTOutputBuf[iIndex++] = 0x7E;
	}

	// ************************************************************************
	// Angular Velocity Bluetooth transmit packet type 3
	// total bytes for packet type 2 is range 0 to 13 = 14 bytes
	// ************************************************************************

	if (globals.AngularVelocityPacketOn)
	{
		// [0]: packet start byte
		sUARTOutputBuf[iIndex++] = 0x7E;

		// [1]: packet type 3 byte (angular velocity)
		tmpuint8 = 0x03;
		sBufAppendItem(sUARTOutputBuf, &iIndex, &tmpuint8, 1);

		// [2]: packet number byte
		sBufAppendItem(sUARTOutputBuf, &iIndex, &(globals.iPacketNumber), 1);
		globals.iPacketNumber++;

		// [6-3]: time stamp (4 bytes)
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&(mqxglobals.FTMTimestamp), 4);

		// [12-7]: add the scaled angular velocity vector to the output buffer
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&iOmega[X], 2);
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&iOmega[Y], 2);
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&iOmega[Z], 2);

		// [13]: add the tail byte for the angular velocity packet type 3
		sUARTOutputBuf[iIndex++] = 0x7E;
	}

	// ************************************************************************
	// Roll, Pitch, Compass Euler angles packet type 4
	// total bytes for packet type 4 is range 0 to 13 = 14 bytes
	// ************************************************************************

	if 	(globals.RPCPacketOn)
	{
		// [0]: packet start byte
		sUARTOutputBuf[iIndex++] = 0x7E;

		// [1]: packet type 4 byte (Euler angles)
		tmpuint8 = 0x04;
		sBufAppendItem(sUARTOutputBuf, &iIndex, &tmpuint8, 1);

		// [2]: packet number byte
		sBufAppendItem(sUARTOutputBuf, &iIndex, &(globals.iPacketNumber), 1);
		globals.iPacketNumber++;

		// [6-3]: time stamp (4 bytes)
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&(mqxglobals.FTMTimestamp), 4);

		// [12-7]: add the angles (resolution 0.1 deg per count) to the transmit buffer
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&iPhi, 2);
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&iThe, 2);
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&iRho, 2);

		// [13]: add the tail byte for the roll, pitch, compass angle packet type 4
		sUARTOutputBuf[iIndex++] = 0x7E;
	}

	// ************************************************************************
	// Altitude / Temperature packet type 5
	// total bytes for packet type 5 is range 0 to 13 = 14 bytes
	// ************************************************************************

#ifdef COMPUTE_1DOF_P_BASIC
	if (globals.AltPacketOn && globals.iMPL3115Found)
	{
		// [0]: packet start byte
		sUARTOutputBuf[iIndex++] = 0x7E;

		// [1]: packet type 5 byte
		tmpuint8 = 0x05;
		sBufAppendItem(sUARTOutputBuf, &iIndex, &tmpuint8, 1);

		// [2]: packet number byte
		sBufAppendItem(sUARTOutputBuf, &iIndex, &(globals.iPacketNumber), 1);
		globals.iPacketNumber++;

		// [6-3]: time stamp (4 bytes)
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&(mqxglobals.FTMTimestamp), 4);

		// [10-7]: altitude (4 bytes, metres times 1000)
		tmpint32 = (int32)(thisSV_1DOF_P_BASIC.fLPHp * 1000.0F);
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint32, 4);

		// [12-11]: temperature (2 bytes, deg C times 100)
		tmpint16 = (int16)(thisSV_1DOF_P_BASIC.fLPTp * 100.0F);
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);

		// [13]: add the tail byte for the altitude / temperature packet type 5
		sUARTOutputBuf[iIndex++] = 0x7E;
	}
#endif

	// ************************************************************************
	// magnetic buffer packet type 6
	// currently total size is 0 to 15 equals 16 bytes 
	// ************************************************************************

#if defined COMPUTE_3DOF_B_BASIC || defined COMPUTE_6DOF_GB_BASIC || defined COMPUTE_9DOF_GBY_KALMAN
	// [0]: packet start byte
	sUARTOutputBuf[iIndex++] = 0x7E;

	// [1]: packet type 6 byte
	tmpuint8 = 0x06;
	sBufAppendItem(sUARTOutputBuf, &iIndex, &tmpuint8, 1);

	// [2]: packet number byte
	sBufAppendItem(sUARTOutputBuf, &iIndex, &(globals.iPacketNumber), 1);
	globals.iPacketNumber++;

	// [4-3]: number of active measurements in the magnetic buffer
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&(thisMagBuffer.iMagBufferCount), 2);	
	
	// [6-5]: fit error (%) with resolution 0.1% 
	if (thisMagCal.fFitErrorpc > 3276.7F)
		tmpint16 = 32767;
	else
		tmpint16 = (int16) (thisMagCal.fFitErrorpc * 10.0F);		
	sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);		

	// always calculate magnetic buffer row and column (low overhead and saves warnings)
	k = globals.MagneticPacketID - 10;
	j = k / MAGBUFFSIZEX;
	i = k - j * MAGBUFFSIZEX;

	// [8-7]: int16: ID of magnetic variable to be transmitted
	// ID 0 to 4 inclusive are magnetic calibration coefficients
	// ID 5 to 9 inclusive are for future expansion
	// ID 10 to (MAGBUFFSIZEX=12) * (MAGBUFFSIZEY=24)-1 or 10 to 10+288-1 are magnetic buffer elements
	// where the convention is used that a negative value indicates empty buffer element (index=-1)
	if ((globals.MagneticPacketID >= 10) && (thisMagBuffer.index[i][j] == -1))
	{
		// use negative ID to indicate inactive magnetic buffer element
		tmpint16 = -globals.MagneticPacketID;
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
	}
	else
	{
		// use positive ID unchanged for variable or active magnetic buffer entry
		tmpint16 = globals.MagneticPacketID;
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
	}

	// [10-9]: int16: variable 1 to be transmitted this iteration
	// [12-11]: int16: variable 2 to be transmitted this iteration
	// [14-13]: int16: variable 3 to be transmitted this iteration
	switch (globals.MagneticPacketID)
	{
	case 0:	
		// item 1: currently unused
		tmpint16 = 0;			
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);		
		// item 2: geomagnetic field strength with resolution 0.1uT
		tmpint16 = (int16)(thisMagCal.fB * 10.0F);
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
		// item 3: magnetic inclination angle with resolution 0.1 deg	
		tmpint16 = iDelta;
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
		break;
	case 1:
		// items 1 to 3: hard iron components range -3276uT to +3276uT encoded with 0.1uT resolution
		tmpint16 = (int16)(thisMagCal.fV[X] * 10.0F);
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
		tmpint16 = (int16)(thisMagCal.fV[Y] * 10.0F);
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);		
		tmpint16 = (int16)(thisMagCal.fV[Z] * 10.0F);
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
		break;			
	case 2:
		// items 1 to 3: diagonal soft iron range -32. to +32. encoded with 0.001 resolution
		tmpint16 = (int16)(thisMagCal.finvW[X][X] * 1000.0F);
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
		tmpint16 = (int16)(thisMagCal.finvW[Y][Y] * 1000.0F);
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
		tmpint16 = (int16)(thisMagCal.finvW[Z][Z] * 1000.0F);
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
		break;
	case 3: 
		// items 1 to 3: off-diagonal soft iron range -32. to +32. encoded with 0.001 resolution
		tmpint16 = (int16)(thisMagCal.finvW[X][Y] * 1000.0F);	
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
		tmpint16 = (int16)(thisMagCal.finvW[X][Z] * 1000.0F);
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
		tmpint16 = (int16)(thisMagCal.finvW[Y][Z] * 1000.0F);
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
		break;
	case 4:				
	case 5:		
	case 6:		
	case 7:		
	case 8:		
	case 9:		
		// cases 4 to 9 inclusive are for future expansion so transmit zeroes for now
		tmpint16 = 0;		
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);		
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);	
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);	
		break;
	default:
		// 10 and upwards: this handles the magnetic buffer elements
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&(thisMagBuffer.iBpFast[X][i][j]), 2);
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&(thisMagBuffer.iBpFast[Y][i][j]), 2);
		sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&(thisMagBuffer.iBpFast[Z][i][j]), 2);
		break;
	}

	// wrap the variable ID back to zero if necessary
	globals.MagneticPacketID++;
	if (globals.MagneticPacketID >= (10 + MAGBUFFSIZEX * MAGBUFFSIZEY))
		globals.MagneticPacketID = 0;

	// [15]: add the tail byte for the magnetic packet type 6
	sUARTOutputBuf[iIndex++] = 0x7E;
#endif
	
	// ************************************************************************
	// Kalman filter packet type 7
	// total bytes for packet type 7 is range 0 to 21 inclusive = 22 bytes
	// ************************************************************************

#if defined COMPUTE_6DOF_GY_KALMAN || defined COMPUTE_9DOF_GBY_KALMAN
	// only send this packet if a Kalman filter quaternion is requested
	if ((globals.QuaternionPacketType == Q6AG) || (globals.QuaternionPacketType == Q9))
	{
		// [0]: packet start byte
		sUARTOutputBuf[iIndex++] = 0x7E;

		// [1]: packet type 7 byte
		tmpuint8 = 0x07;
		sBufAppendItem(sUARTOutputBuf, &iIndex, &tmpuint8, 1);

		// [2]: packet number byte
		sBufAppendItem(sUARTOutputBuf, &iIndex, &(globals.iPacketNumber), 1);
		globals.iPacketNumber++;

		// [4-3]: fThErrPl[X] resolution 0.001 deg
		// [6-5]: fThErrPl[Y] resolution 0.001 deg
		// [8-7]: fThErrPl[Z] resolution 0.001 deg
		for (i = X; i <= Z; i++)
		{
			ftmp = 0.0F;
#if defined COMPUTE_6DOF_GY_KALMAN
			if (globals.QuaternionPacketType == Q6AG)
				ftmp = thisSV_6DOF_GY_KALMAN.fThErrPl[i] * 1000.0F;
#endif
#if defined COMPUTE_9DOF_GBY_KALMAN
			if (globals.QuaternionPacketType == Q9)
				ftmp = thisSV_9DOF_GBY_KALMAN.fThErrPl[i] * 1000.0F;
#endif
			// check for clipping
			if (ftmp > 32767.0F)
				tmpint16 = 32767;
			else if (ftmp < -32768.0F)
				tmpint16 = -32768;
			else
				tmpint16 = (int16) ftmp;

			// add the data to the packet
			sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
		}

		// [10-9]: fbPl[X] resolution 0.001 deg/sec
		// [12-11]: fbPl[Y] resolution 0.001 deg/sec
		// [14-13]: fbPl[Z] resolution 0.001 deg/sec
		for (i = X; i <= Z; i++)
		{
			ftmp = 0.0F;
#if defined COMPUTE_6DOF_GY_KALMAN
			if (globals.QuaternionPacketType == Q6AG)
				ftmp = (thisSV_6DOF_GY_KALMAN.fbPl[i] * 1000.0F);
#endif
#if defined COMPUTE_9DOF_GBY_KALMAN
			if (globals.QuaternionPacketType == Q9)
				ftmp = (thisSV_9DOF_GBY_KALMAN.fbPl[i] * 1000.0F);
#endif
			// check for clipping
			if (ftmp > 32767.0F)
				tmpint16 = 32767;
			else if (ftmp < -32768.0F)
				tmpint16 = -32768;
			else
				tmpint16 = (int16) ftmp;

			// add the data to the packet
			sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
		}

		// [16-15]: fbErrPl[X] resolution 0.0001deg/s
		// [18-17]: fbErrPl[Y] resolution 0.0001deg/s
		// [20-19]: fbErrPl[Z] resolution 0.0001deg/s		
		for (i = X; i <= Z; i++)
		{
			ftmp = 0.0F;
#if defined COMPUTE_6DOF_GY_KALMAN
			if (globals.QuaternionPacketType == Q6AG)
				ftmp = (thisSV_6DOF_GY_KALMAN.fbErrPl[i] * 10000.0F);
#endif
#if defined COMPUTE_9DOF_GBY_KALMAN
			if (globals.QuaternionPacketType == Q9)
				ftmp = (thisSV_9DOF_GBY_KALMAN.fbErrPl[i] * 10000.0F);
#endif
			// check for clipping
			if (ftmp > 32767.0F)
				tmpint16 = 32767;
			else if (ftmp < -32768.0F)
				tmpint16 = -32768;
			else
				tmpint16 = (int16) ftmp;

			// add the data to the packet
			sBufAppendItem(sUARTOutputBuf, &iIndex, (uint8*)&tmpint16, 2);
		}

		// [21]: add the tail byte for the Kalman packet type 7
		sUARTOutputBuf[iIndex++] = 0x7E;

	} // end of check for Kalman filter quaternion
#endif // end of check for Kalman packet

	// ************************************************************************
	// all packets have now been constructed in the output buffer so
	// transmit over UART to the Bluetooth module
	// the final iIndex++ gives the number of bytes to transmit
	// which is one more than the last index in the buffer.
	// this function is non-blocking
	// ************************************************************************

	UART_SendBlock(DeviceDataPtr, sUARTOutputBuf, iIndex);

	return;
}
