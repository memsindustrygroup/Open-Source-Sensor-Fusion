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
// This file contains build controls for a sensor fusion project.
// Board and MCU customization is done via Processor Expert.  The fusion
// code sits above that software layer, and can usually be ported from
// one board environment to another with no changes.

#ifndef BUILD_H
#define BUILD_H

// PCB HAL options
#define BOARD_WIN8_REV05     0			// with sensor shield
#define BOARD_FRDM_KL25Z     1			// with sensor shield
#define BOARD_FRDM_K20D50M   2			// with sensor shield
#define BOARD_FXLC95000CL    3
#define BOARD_FRDM_KL26Z     4			// with sensor shield
#define BOARD_FRDM_K64F	     5			// with sensor shield
#define BOARD_FRDM_KL16Z     6			// with sensor shield
#define BOARD_FRDM_KL46Z     7			// with sensor shield
#define BOARD_FRDM_KL46Z_STANDALONE	 8	// without sensor shield

// enter new PCBs here with incrementing values
// C Compiler Preprocessor define in the CodeWarrior project will choose which board to use
#ifdef REV05
#define THIS_BOARD_ID		BOARD_WIN8_REV05
#endif
#ifdef KL25Z
#define THIS_BOARD_ID       	BOARD_FRDM_KL25Z		
#endif
#ifdef K20D50M
#define THIS_BOARD_ID		BOARD_FRDM_K20D50M
#endif
#ifdef FXLC95000CL
#define THIS_BOARD_ID		BOARD_FRDM_FXLC95000CL
#endif
#ifdef KL26Z
#define THIS_BOARD_ID		BOARD_FRDM_KL26Z
#endif
#ifdef K64F
#define THIS_BOARD_ID		BOARD_FRDM_K64F
#endif
#ifdef KL16Z
#define THIS_BOARD_ID		BOARD_FRDM_KL16Z
#endif
#ifdef KL46Z
#define THIS_BOARD_ID		BOARD_FRDM_KL46Z
#endif
#ifdef KL46Z_STANDALONE
#define THIS_BOARD_ID		BOARD_FRDM_KL46Z_STANDALONE
#endif

// coordinate system for the build
#define NED 0                       // identifier for NED angle output
#define ANDROID 1                   // identifier for Android angle output
#define WIN8 2						// identifier for Windows 8 angle output
#define THISCOORDSYSTEM ANDROID		// the coordinate system to be used

// sensors to be enabled: compile errors will warn if the sensors are not compatible with the algorithms.
// avoid enabling FXOS8700 plus MMA8652 and MAG3110 which will result in sensor read from all sensors
// with the data read first from FXOS8700 and then over-written by data from MMA8652 and MAG3110.
// it will still work but it's a waste of clock cycles.
#define USE_MPL3115
#define USE_FXOS8700
#define USE_FXAS21000
//#define USE_FXAS21002
//#define USE_MMA8652
//#define USE_MAG3110

// enforce a fatal compilation error if the K20D50M board is used with MMA8652
#if (THIS_BOARD_ID == BOARD_FRDM_K20D50M) && defined USE_MMA8652
#error This build creates an I2C conflict between MMA8451 on K20D50M board and MMA8652 on sensor board
#endif

// set this value to set parallel (1) or sequential (0) operation of fusion algorithms
// a) if defined to be 1 (true) (default for demos) then all algorithms execute
// in parallel but only globals.QuaternionPacketType is transmitted
// b) if defined to be 0 (false) then only the one algorithm required to support
// globals.QuaternionPacketType is executed
#define PARALLELNOTSEQUENTIAL 1		// default is 1

// normally all enabled: degrees of freedom algorithms to be executed
#define COMPUTE_1DOF_P_BASIC		// 1DOF pressure (altitude) and temperature: (1x pressure)
#define COMPUTE_3DOF_G_BASIC		// 3DOF accel tilt: (1x accel)
#define COMPUTE_3DOF_B_BASIC		// 3DOF mag eCompass (vehicle): (1x mag)
#define COMPUTE_3DOF_Y_BASIC		// 3DOF gyro integration: (1x gyro)
#define COMPUTE_6DOF_GB_BASIC		// 6DOF accel and mag eCompass: (1x accel + 1x mag)
#define COMPUTE_6DOF_GY_KALMAN		// 6DOF accel and gyro (Kalman): (1x accel + 1x gyro)
#define COMPUTE_9DOF_GBY_KALMAN		// 9DOF accel, mag and gyro (Kalman): (1x accel + 1x mag + 1x gyro)

// int16 build number sent in Bluetooth debug packet
#define THISBUILD  422

// sampling rate and kalman filter timing
#define FTM_INCLK_HZ		1000000		// int32: 1MHz FTM timer frequency set in PE: do not change
#define SENSORFS 			200         // int32: 200Hz: frequency (Hz) of sensor sampling process
#define OVERSAMPLE_RATIO 	8       	// int32: 8x: 3DOF, 6DOF, 9DOF run at SENSORFS / OVERSAMPLE_RATIO Hz

// power saving deep sleep
//#define DEEPSLEEP				// define to enable deep sleep power saving

// UART (Bluetooth) serial port control
//#define UART_OFF				// define to measure MCU+algorithm current only

// vector components
#define X 0
#define Y 1
#define Z 2

// booleans
#define true 1
#define false 0

// geomagnetic model parameters
#define DEFAULTB 50.0F				// default geomagnetic field (uT)

// useful multiplicative conversion constants
#define PI 3.141592654F					// Pi
#define FDEGTORAD 0.01745329251994F		// degrees to radians conversion = pi / 180
#define FRADTODEG 57.2957795130823F		// radians to degrees conversion = 180 / pi
#define FRECIP180 0.0055555555555F		// multiplicative factor 1/180
#define ONETHIRD 0.33333333F			// one third
#define ONESIXTH 0.166666667F			// one sixth
#define ONETWELFTH 0.0833333333F		// one twelfth
#define ONEOVER48 0.02083333333F		// 1 / 48
#define ONEOVER120 0.0083333333F		// 1 / 120
#define ONEOVER3840 0.0002604166667F	// 1 / 3840
#define ONEOVERROOT2 0.707106781F		// 1/sqrt(2)
#define ROOT3OVER2 0.866025403784F		// sqrt(3)/2

// type definitions
// these re-define (but with no changes) those in MQX-Lite PE-Types.h for Kinetis
typedef signed char				int8;
typedef unsigned char			uint8;
typedef signed short int		int16;
typedef unsigned short int		uint16;
typedef signed long int			int32;
typedef unsigned long int		uint32;
// the quaternion type to be transmitted
typedef enum quaternion {Q3, Q3M, Q3G, Q6MA, Q6AG, Q9} quaternion_type;

// quaternion structure definition
struct fquaternion
{
	float q0;	// scalar component
	float q1;	// x vector component
	float q2;	// y vector component
	float q3;	// z vector component
};

// gyro sensor structure definition
struct PressureSensor
{
	int32 iHpFast;					// fast (typically 200Hz) height (counts)
	int32 iHp;						// slow (typically 25Hz) height (counts)
	float fHp;						// slow (typically 25Hz) height (m)
	float fTp;						// slow (typically 25Hz) temperature (C)
	float fmPerCount;				// initialized to FMPERCOUNT
	float fCPerCount;				// initialized to FCPERCPOUNT
	int16 iTp;						// slow (typically 25Hz) temperature (count)
	int16 iTpFast;					// fast (typically 200Hz) temperature (counts)
};
	
// accelerometer sensor structure definition
struct AccelSensor
{
	int32 iSumGpFast[3];	// sum of fast measurements
	float fGpFast[3];		// fast (typically 200Hz) readings (g)
	float fGp[3];			// slow (typically 25Hz) averaged readings (g)
	float fgPerCount;		// initialized to FGPERCOUNT
	int16 iGpFast[3];		// fast (typically 200Hz) readings
	int16 iGp[3];			// slow (typically 25Hz) averaged readings (counts)
};

// magnetometer sensor structure definition
struct MagSensor
{
	int32 iSumBpFast[3];	// sum of fast measurements
	float fBpFast[3];		// fast (typically 200Hz) raw readings (uT)
	float fBp[3];			// slow (typically 25Hz) averaged raw readings (uT)
	float fBcFast[3];		// fast (typically 200Hz) calibrated readings (uT)
	float fBc[3];			// slow (typically 25Hz) averaged calibrated readings (uT)
	float fuTPerCount;		// initialized to FUTPERCOUNT
	float fCountsPeruT;		// initialized to FCOUNTSPERUT
	int16 iBpFast[3];		// fast (typically 200Hz) raw readings (counts)
	int16 iBp[3];			// slow (typically 25Hz) averaged raw readings (counts)
	int16 iBc[3];			// slow (typically 25Hz) averaged calibrated readings (counts)
};

// gyro sensor structure definition
struct GyroSensor
{
	int32 iSumYpFast[3];					// sum of fast measurements
	float fYp[3];							// raw gyro sensor output (deg/s)
	float fDegPerSecPerCount;				// initialized to FDEGPERSECPERCOUNT
	int16 iYpFast[OVERSAMPLE_RATIO][3];		// fast (typically 200Hz) readings
	int16 iYp[3];							// averaged gyro sensor output (counts)
};

#endif // BUILD_H
