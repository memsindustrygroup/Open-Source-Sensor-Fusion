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
// This file defines sensor fusion apps in terms of very high level functions.
// It also includes functions for applying hardware abstraction mapping (HAL functions).
// These include:
//    RdSensData_Init()
//    Fusion_Init()
//    RdSensData_Run()
//    Fusion_Run()
//    MagCal_Run()
//    ApplyAccelHAL()
//    ApplyMagHAL()
//    ApplyGyroHAL()

#include "Events.h"
#include "include_all.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "time.h"
#include "string.h"

// sensor data structures
#if defined USE_MPL3115
struct PressureSensor thisPressure;			// this pressure sensor
#endif
#if defined USE_FXOS8700 || defined USE_MMA8652
struct AccelSensor thisAccel;				// this accelerometer
#endif
#if defined USE_FXOS8700 || defined USE_MAG3110
struct MagSensor thisMag;					// this magnetometer
struct MagCalibration thisMagCal;			// hard and soft iron magnetic calibration
struct MagneticBuffer thisMagBuffer;		// magnetometer measurement buffer
#endif
#if defined USE_FXAS21000 || defined USE_FXAS21002
struct GyroSensor thisGyro;					// this gyro
#endif

// 1DOF pressure structure
#if defined COMPUTE_1DOF_P_BASIC
struct SV_1DOF_P_BASIC thisSV_1DOF_P_BASIC;					
#endif

// 3DOF accelerometer (Basic) structure
#if defined COMPUTE_3DOF_G_BASIC
struct SV_3DOF_G_BASIC thisSV_3DOF_G_BASIC;
#endif

// 3DOF magnetometer (Basic) structure
#if defined COMPUTE_3DOF_B_BASIC
struct SV_3DOF_B_BASIC thisSV_3DOF_B_BASIC;					
#endif

// 3DOF gyro (Basic) structure
#if defined COMPUTE_3DOF_Y_BASIC
struct SV_3DOF_Y_BASIC thisSV_3DOF_Y_BASIC;					
#endif

// 6DOF accelerometer and magnetometer (Basic) structure
#if defined COMPUTE_6DOF_GB_BASIC
struct SV_6DOF_GB_BASIC thisSV_6DOF_GB_BASIC;					
#endif

// 6DOF accelerometer and gyro (Kalman) structure
#if defined COMPUTE_6DOF_GY_KALMAN
struct SV_6DOF_GY_KALMAN thisSV_6DOF_GY_KALMAN;
#endif

// 9DOF accelerometer, magnetometer and gyro (Kalman) structure
#if defined COMPUTE_9DOF_GBY_KALMAN
struct SV_9DOF_GBY_KALMAN thisSV_9DOF_GBY_KALMAN;
#endif

// function initializes the sensors and the sensor data structures
void RdSensData_Init(void)
{
	int8 i;							// loop counter

	// zero sums of sensor data (typically 200Hz) on first execution
	for (i = X; i <= Z; i++)
	{
#if defined USE_FXOS8700 || defined USE_MMA8652
		thisAccel.iSumGpFast[i] = 0;
#endif
#if defined USE_FXOS8700 || defined USE_MAG3110
		thisMag.iSumBpFast[i] = 0;
#endif
#if defined USE_FXAS21000 || defined USE_FXAS21002
		thisGyro.iSumYpFast[i] = 0;
#endif
	}

	// initialize the physical sensors over I2C
#if defined USE_MPL3115
	globals.iMPL3115Found = MPL3115_Init(I2C_DeviceData, &thisPressure);
#endif
#if defined USE_FXOS8700
	FXOS8700_Init(I2C_DeviceData, &thisAccel, &thisMag);
#endif
#if defined USE_FXAS21000
	FXAS21000_Init(I2C_DeviceData, &thisGyro);
#endif
#if defined USE_FXAS21002
	FXAS21002_Init(I2C_DeviceData, &thisGyro);
#endif
#if defined USE_MMA8652
	MMA8652_Init(I2C_DeviceData, &thisAccel);
#endif
#if defined USE_MAG3110
	MAG3110_Init(I2C_DeviceData, &thisMag);
#endif

	return;
}

// function initializes the sensor fusion and magnetic calibration and sets loopcounter to zero
void Fusion_Init(void)
{
	// magnetic DOF: reset magnetic calibration and magnetometer data buffer (not needed for 3DOF)
#if defined COMPUTE_3DOF_B_BASIC || defined COMPUTE_6DOF_GB_BASIC || defined COMPUTE_9DOF_GBY_KALMAN
	fInitMagCalibration(&thisMagCal, &thisMagBuffer);
#endif

	// reset the default quaternion type to the simplest Q3 (it will be updated during the initializations)
	globals.DefaultQuaternionPacketType = Q3;
	
	// force a reset of all the algorithms next time they execute
	// the initialization will result in the default and current quaternion being set to the most sophisticated
	// algorithm supported by the build
#if defined COMPUTE_1DOF_P_BASIC
	thisSV_1DOF_P_BASIC.resetflag = true;					
#endif
#if defined COMPUTE_3DOF_G_BASIC
	thisSV_3DOF_G_BASIC.resetflag = true;				
#endif
#if defined COMPUTE_3DOF_B_BASIC
	thisSV_3DOF_B_BASIC.resetflag = true;	
#endif
#if defined COMPUTE_3DOF_Y_BASIC
	thisSV_3DOF_Y_BASIC.resetflag = true;				
#endif
#if defined COMPUTE_6DOF_GB_BASIC
	thisSV_6DOF_GB_BASIC.resetflag = true;				
#endif
#if defined COMPUTE_6DOF_GY_KALMAN
	thisSV_6DOF_GY_KALMAN.resetflag = true;
#endif
#if defined COMPUTE_9DOF_GBY_KALMAN
	thisSV_9DOF_GBY_KALMAN.resetflag = true;
#endif

	// reset the loop counter to zero for first iteration
	globals.loopcounter = 0;

	return;
}

// this function is called at (typically 200Hz) by MQX to read sensor data
void RdSensData_Run(void)
{
	static int8 iCounter = 0;		// the number of gyro readings summed
	int8 i;							// loop counter

	// read the 200Hz sensor data and apply the HAL
#if defined USE_MPL3115
	if (globals.iMPL3115Found)
	{
		MPL3115_ReadData(I2C_DeviceData, &thisPressure);
	}
#endif
#if defined USE_FXOS8700
	FXOS8700_ReadData(I2C_DeviceData, &thisAccel, &thisMag);
	ApplyAccelHAL(&thisAccel);
	ApplyMagHAL(&thisMag);
#endif
#if defined USE_FXAS21000
	FXAS21000_ReadData(I2C_DeviceData, &thisGyro, iCounter);
	ApplyGyroHAL(&thisGyro, iCounter);
#endif	
#if defined USE_FXAS21002
	FXAS21002_ReadData(I2C_DeviceData, &thisGyro, iCounter);
	ApplyGyroHAL(&thisGyro, iCounter);
#endif
#if defined USE_MMA8652
	MMA8652_ReadData(I2C_DeviceData, &thisAccel);
	ApplyAccelHAL(&thisAccel);
#endif
#if defined USE_MAG3110
	MAG3110_ReadData(I2C_DeviceData, &thisMag);
	ApplyMagHAL(&thisMag);
#endif

	// sum the 200Hz HAL-aligned sensor data
	for (i = X; i <= Z; i++)
	{
#if defined USE_FXOS8700 || defined USE_MMA8652
		thisAccel.iSumGpFast[i] += (int32) thisAccel.iGpFast[i];
#endif
#if defined USE_FXOS8700 || defined USE_MAG3110
		thisMag.iSumBpFast[i] += (int32) thisMag.iBpFast[i];
#endif
#if defined USE_FXAS21000 || defined USE_FXAS21002
		thisGyro.iSumYpFast[i] += (int32) thisGyro.iYpFast[iCounter][i];
#endif
	}

	// increment the decimation counter for the next iteration
	iCounter++;

	// every OVERSAMPLE_RATIO iterations process the summed over-sampled readings
	if (iCounter == OVERSAMPLE_RATIO)
	{
#if defined USE_MPL3115
		// for pressure and temperature, use the most recent measurement in preference to averaging
		// since the MPL3115 is configured for maximum 512 sample over-sampling already
		thisPressure.iHp = thisPressure.iHpFast;;
		thisPressure.iTp = thisPressure.iTpFast;
		thisPressure.fHp = (float) thisPressure.iHp * thisPressure.fmPerCount;
		thisPressure.fTp = (float) thisPressure.iTp * thisPressure.fCPerCount;
#endif

		// process the HAL-aligned sensor measurements prior to calling the sensor fusion
		for (i = X; i <= Z; i++)
		{
#if defined USE_FXOS8700 || defined USE_MMA8652
			// calculate the fast accelerometer reading for the Kalman filters (to reduce phase errors)
			thisAccel.fGpFast[i] = (float) thisAccel.iGpFast[i] * thisAccel.fgPerCount;
			// calculate the average (typically 25Hz) accelerometer reading
			thisAccel.iGp[i] = (int16)(thisAccel.iSumGpFast[i] / OVERSAMPLE_RATIO);
			thisAccel.fGp[i] = (float) thisAccel.iSumGpFast[i] * thisAccel.fgPerCount / (float) OVERSAMPLE_RATIO;
#endif
#if defined USE_FXOS8700 || defined USE_MAG3110
			// calculate the fast magnetometer reading for the Kalman filters (to reduce phase errors)
			thisMag.fBpFast[i] = (float) thisMag.iBpFast[i] * thisMag.fuTPerCount;
			// calculate the average (typically 25Hz) magnetometer reading
			thisMag.iBp[i] = (int16)(thisMag.iSumBpFast[i] / OVERSAMPLE_RATIO);
			thisMag.fBp[i] = (float) thisMag.iSumBpFast[i] * thisMag.fuTPerCount / (float) OVERSAMPLE_RATIO;
#endif
#if defined USE_FXAS21000 || defined USE_FXAS21002
			// calculate the average (typically 25Hz) gyro reading
			thisGyro.iYp[i] = (int16)(thisGyro.iSumYpFast[i] / OVERSAMPLE_RATIO);
			thisGyro.fYp[i] = (float) thisGyro.iSumYpFast[i] * thisGyro.fDegPerSecPerCount / (float) OVERSAMPLE_RATIO;
#endif
			// zero the sensor sums for the next iteration
#if defined USE_FXOS8700 || defined USE_MMA8652
			thisAccel.iSumGpFast[i] = 0;
#endif
#if defined USE_FXOS8700 || defined USE_MAG3110
			thisMag.iSumBpFast[i] = 0;
#endif
#if defined USE_FXAS21000 || defined USE_FXAS21002
			thisGyro.iSumYpFast[i] = 0;
#endif
		}

		// zero the counter and set the event flag to start the sensor fusion task
		iCounter = 0;
		mqxglobals.RunKF_Event_Flag = 1;

	} // end of over-sampling test
}

// function runs the sensor fusion algorithms
void Fusion_Run(void)
{
	int8 initiatemagcal;				// flag to initiate a new magnetic calibration

#if defined COMPUTE_3DOF_B_BASIC || defined COMPUTE_6DOF_GB_BASIC || defined COMPUTE_9DOF_GBY_KALMAN
	// magnetic DOF: remove hard and soft iron terms from Bp (uT) to get calibrated data Bc (uT)
	fInvertMagCal(&thisMag, &thisMagCal);

	// update magnetic buffer checking for i) absence of first all-zero magnetometer output and ii) no calibration in progress
	// an all zero magnetometer reading can occur after power-on at rare intervals but it simply won't be used in the buffer
	if (!((globals.loopcounter < 100) && (thisMag.iBpFast[X] == 0) && (thisMag.iBpFast[Y] == 0) && (thisMag.iBpFast[Z] == 0)) && !thisMagCal.iCalInProgress)
	{
		// update the magnetometer measurement buffer integer magnetometer data (typically at 25Hz)
		iUpdateMagnetometerBuffer(&thisMagBuffer, &thisAccel, &thisMag, globals.loopcounter);
	}
#endif

	// 1DOF Pressure low pass filter algorithm
#if defined COMPUTE_1DOF_P_BASIC
	thisSV_1DOF_P_BASIC.systick = SYST_CVR & 0x00FFFFFF;
	fRun_1DOF_P_BASIC(&thisSV_1DOF_P_BASIC, &thisPressure, globals.loopcounter);
	thisSV_1DOF_P_BASIC.systick -= SYST_CVR & 0x00FFFFFF;
	if (thisSV_1DOF_P_BASIC.systick < 0) thisSV_1DOF_P_BASIC.systick += SYST_RVR;	
#endif

	// 3DOF Accel Basic: call the tilt algorithm, low pass filters and Euler angle calculation
#if defined COMPUTE_3DOF_G_BASIC
	if (PARALLELNOTSEQUENTIAL || (globals.QuaternionPacketType == Q3))
	{
		thisSV_3DOF_G_BASIC.systick = SYST_CVR & 0x00FFFFFF;		
		fRun_3DOF_G_BASIC(&thisSV_3DOF_G_BASIC, &thisAccel, globals.loopcounter, THISCOORDSYSTEM);
		thisSV_3DOF_G_BASIC.systick -= SYST_CVR & 0x00FFFFFF;
		if (thisSV_3DOF_G_BASIC.systick < 0) thisSV_3DOF_G_BASIC.systick += SYST_RVR;
	}
#endif

	// 3DOF Magnetometer Basic: call the 2D vehicle compass algorithm
#if defined COMPUTE_3DOF_B_BASIC
	if (PARALLELNOTSEQUENTIAL || (globals.QuaternionPacketType == Q3M))
	{
		thisSV_3DOF_B_BASIC.systick = SYST_CVR & 0x00FFFFFF;
		fRun_3DOF_B_BASIC(&thisSV_3DOF_B_BASIC, &thisMag, globals.loopcounter, THISCOORDSYSTEM);
		thisSV_3DOF_B_BASIC.systick -= SYST_CVR & 0x00FFFFFF;
		if (thisSV_3DOF_B_BASIC.systick < 0) thisSV_3DOF_B_BASIC.systick += SYST_RVR;
	}
#endif

	// 3DOF Gyro Basic: call the gyro integration algorithm
#if defined COMPUTE_3DOF_Y_BASIC
	if (PARALLELNOTSEQUENTIAL || (globals.QuaternionPacketType == Q3G))
	{
		thisSV_3DOF_Y_BASIC.systick = SYST_CVR & 0x00FFFFFF;
		fRun_3DOF_Y_BASIC(&thisSV_3DOF_Y_BASIC, &thisGyro, globals.loopcounter, THISCOORDSYSTEM,
				OVERSAMPLE_RATIO);
		thisSV_3DOF_Y_BASIC.systick -= SYST_CVR & 0x00FFFFFF;
		if (thisSV_3DOF_Y_BASIC.systick < 0) thisSV_3DOF_Y_BASIC.systick += SYST_RVR;
	}
#endif

	// 6DOF Accel / Mag: Basic: call the eCompass orientation algorithm, low pass filters and Euler angle calculation
#if defined COMPUTE_6DOF_GB_BASIC
	if (PARALLELNOTSEQUENTIAL || (globals.QuaternionPacketType == Q6MA))
	{
		thisSV_6DOF_GB_BASIC.systick = SYST_CVR & 0x00FFFFFF;
		fRun_6DOF_GB_BASIC(&thisSV_6DOF_GB_BASIC, &thisMag, &thisAccel, globals.loopcounter, THISCOORDSYSTEM);
		thisSV_6DOF_GB_BASIC.systick -= SYST_CVR & 0x00FFFFFF;
		if (thisSV_6DOF_GB_BASIC.systick < 0) thisSV_6DOF_GB_BASIC.systick += SYST_RVR;
	}
#endif

	// 6DOF Accel / Gyro: call the Kalman orientation algorithm
#if defined COMPUTE_6DOF_GY_KALMAN
	if (PARALLELNOTSEQUENTIAL || (globals.QuaternionPacketType == Q6AG))
	{
		thisSV_6DOF_GY_KALMAN.systick = SYST_CVR & 0x00FFFFFF;
		fRun_6DOF_GY_KALMAN(&thisSV_6DOF_GY_KALMAN, &thisAccel,  &thisGyro, THISCOORDSYSTEM, OVERSAMPLE_RATIO);
		thisSV_6DOF_GY_KALMAN.systick -= SYST_CVR & 0x00FFFFFF;
		if (thisSV_6DOF_GY_KALMAN.systick < 0) thisSV_6DOF_GY_KALMAN.systick += SYST_RVR;
	}
#endif

	// 9DOF Accel / Mag / Gyro: apply the Kalman filter
#if defined COMPUTE_9DOF_GBY_KALMAN
	if (PARALLELNOTSEQUENTIAL || (globals.QuaternionPacketType == Q9))
	{
		thisSV_9DOF_GBY_KALMAN.systick = SYST_CVR & 0x00FFFFFF;
		fRun_9DOF_GBY_KALMAN(&thisSV_9DOF_GBY_KALMAN, &thisAccel, &thisMag, &thisGyro, &thisMagCal, THISCOORDSYSTEM, OVERSAMPLE_RATIO);
		thisSV_9DOF_GBY_KALMAN.systick -= SYST_CVR & 0x00FFFFFF;		
		if (thisSV_9DOF_GBY_KALMAN.systick < 0) thisSV_9DOF_GBY_KALMAN.systick += SYST_RVR;
	}
#endif // COMPUTE_9DOF_GBY_KALMAN

	// 6DOF and 9DOF: decide whether to initiate a magnetic calibration
#if defined COMPUTE_3DOF_B_BASIC || defined COMPUTE_6DOF_GB_BASIC || defined COMPUTE_9DOF_GBY_KALMAN
	// check no magnetic calibration is in progress
	if (!thisMagCal.iCalInProgress)
	{
		// do the first 4 element calibration immediately there are a minimum of MINMEASUREMENTS4CAL
		initiatemagcal = (!thisMagCal.iMagCalHasRun && (thisMagBuffer.iMagBufferCount >= MINMEASUREMENTS4CAL));

		// otherwise initiate a calibration at intervals depending on the number of measurements available
		initiatemagcal |= ((thisMagBuffer.iMagBufferCount >= MINMEASUREMENTS4CAL) && 
				(thisMagBuffer.iMagBufferCount < MINMEASUREMENTS7CAL) &&
				!(globals.loopcounter % INTERVAL4CAL));
		initiatemagcal |= ((thisMagBuffer.iMagBufferCount >= MINMEASUREMENTS7CAL) &&
				(thisMagBuffer.iMagBufferCount < MINMEASUREMENTS10CAL) &&
				!(globals.loopcounter % INTERVAL7CAL));
		initiatemagcal |= ((thisMagBuffer.iMagBufferCount >= MINMEASUREMENTS10CAL) &&
				!(globals.loopcounter % INTERVAL10CAL));

		// initiate the magnetic calibration if any of the conditions are met
		if (initiatemagcal)
		{
			// set the flags denoting that a calibration is in progress
			thisMagCal.iCalInProgress = 1;
			thisMagCal.iMagCalHasRun = 1;

			// enable the magnetic calibration task to run
			mqxglobals.MagCal_Event_Flag = 1;
		} // end of test whether to call calibration functions
	} // end of test that no calibration is already in progress
#endif

	// increment the loopcounter (used for time stamping magnetic data)
	globals.loopcounter++;

	return;
}

// function runs the magnetic calibration
void MagCal_Run(struct MagCalibration *pthisMagCal, struct MagneticBuffer *pthisMagBuffer)
{
	int8 i, j;			// loop counters
	int8 isolver;		// magnetic solver used

	// 4 element calibration case
	if (pthisMagBuffer->iMagBufferCount < MINMEASUREMENTS7CAL)
	{
		// age the existing fit error to avoid one good calibration locking out future updates
		if (pthisMagCal->iValidMagCal)
		{
			pthisMagCal->fFitErrorpc *= (1.0F + (float) INTERVAL4CAL * (float) OVERSAMPLE_RATIO / ((float) SENSORFS * FITERRORAGINGSECS));	
		}
		// call the 4 element matrix inversion calibration
		isolver = 4;
		fUpdateCalibration4INV(pthisMagCal, pthisMagBuffer, &thisMag);
	}
	// 7 element calibration case
	else if (pthisMagBuffer->iMagBufferCount < MINMEASUREMENTS10CAL)
	{
		// age the existing fit error to avoid one good calibration locking out future updates
		if (pthisMagCal->iValidMagCal)
		{
			pthisMagCal->fFitErrorpc *= (1.0F + (float) INTERVAL7CAL * (float) OVERSAMPLE_RATIO / ((float) SENSORFS * FITERRORAGINGSECS));	
		}
		// call the 7 element eigenpair calibration
		isolver = 7;
		fUpdateCalibration7EIG(pthisMagCal, pthisMagBuffer, &thisMag);
	}
	// 10 element calibration case
	else
	{
		// age the existing fit error to avoid one good calibration locking out future updates
		if (pthisMagCal->iValidMagCal)
		{
			pthisMagCal->fFitErrorpc *= (1.0F + (float) INTERVAL10CAL * (float) OVERSAMPLE_RATIO / ((float) SENSORFS * FITERRORAGINGSECS));	
		}
		// call the 10 element eigenpair calibration
		isolver = 10;
		fUpdateCalibration10EIG(pthisMagCal, pthisMagBuffer, &thisMag);
	}

	// the trial geomagnetic field must be in range (earth is 22uT to 67uT)
	if ((pthisMagCal->ftrB >= MINBFITUT) && (pthisMagCal->ftrB <= MAXBFITUT))		
	{
		// always accept the calibration if i) no previous calibration exists ii) the calibration fit is reduced or
		// an improved solver was used giving a good trial calibration (4% or under)
		if ((pthisMagCal->iValidMagCal == 0) ||
				(pthisMagCal->ftrFitErrorpc <= pthisMagCal->fFitErrorpc) ||
				((isolver > pthisMagCal->iValidMagCal) && (pthisMagCal->ftrFitErrorpc <= 4.0F)))
		{
			// accept the new calibration solution
			pthisMagCal->iValidMagCal = isolver;
			pthisMagCal->fFitErrorpc = pthisMagCal->ftrFitErrorpc;
			pthisMagCal->fB = pthisMagCal->ftrB;
			pthisMagCal->fFourBsq = 4.0F * pthisMagCal->ftrB * pthisMagCal->ftrB;
			for (i = X; i <= Z; i++)
			{
				pthisMagCal->fV[i] = pthisMagCal->ftrV[i];
				for (j = X; j <= Z; j++)
				{
					pthisMagCal->finvW[i][j] = pthisMagCal->ftrinvW[i][j];
				}
			}
		} // end of test to accept the new calibration 
	} // end of test for geomagenetic field strength in range

	// reset the calibration in progress flag to allow writing to the magnetic buffer
	pthisMagCal->iCalInProgress = 0;

	return;
}

// function applies the hardware abstraction layer to the Fast (typically 200Hz) accelerometer readings
void ApplyAccelHAL(struct AccelSensor *pthisAccel)
{
#if THISCOORDSYSTEM == NED
	int16 itmp16;
	itmp16 = thisAccel.iGpFast[X];
	thisAccel.iGpFast[X] = thisAccel.iGpFast[Y];
	thisAccel.iGpFast[Y] = itmp16;
#endif // NED
#if THISCOORDSYSTEM == ANDROID
	thisAccel.iGpFast[X] = -thisAccel.iGpFast[X];
	thisAccel.iGpFast[Y] = -thisAccel.iGpFast[Y];
#endif // Android
#if (THISCOORDSYSTEM == WIN8)
	thisAccel.iGpFast[Z] = -thisAccel.iGpFast[Z];
#endif // Win8

	return;
}

// function applies the hardware abstraction layer to the Fast (typically 200Hz) magnetometer readings
void ApplyMagHAL(struct MagSensor *pthisMag)
{
#if THISCOORDSYSTEM == NED
	int16 itmp16;
	itmp16 = thisMag.iBpFast[X];
	thisMag.iBpFast[X] = -thisMag.iBpFast[Y];
	thisMag.iBpFast[Y] = -itmp16;
	thisMag.iBpFast[Z] = -thisMag.iBpFast[Z];
#endif // NED
#if THISCOORDSYSTEM == ANDROID
	thisMag.iBpFast[X] = -thisMag.iBpFast[X];
	thisMag.iBpFast[Y] = -thisMag.iBpFast[Y];	
#endif // Android
#if THISCOORDSYSTEM == WIN8
	thisMag.iBpFast[X] = -thisMag.iBpFast[X];
	thisMag.iBpFast[Y] = -thisMag.iBpFast[Y];
#endif

	// finally correct for the left handed magnetic coordinate system in MAG3110
#if defined USE_MAG3110
	thisMag.iBpFast[Z] = -thisMag.iBpFast[Z];
#endif

	return;
}

// function applies the hardware abstraction layer to the Fast (typically 200Hz) gyro readings
void ApplyGyroHAL(struct GyroSensor *pthisGyro, int16 irow)
{
#if THISCOORDSYSTEM == NED
	int16 itmp16;
	itmp16 = thisGyro.iYpFast[irow][X];
	thisGyro.iYpFast[irow][X] = -thisGyro.iYpFast[irow][Y];
	thisGyro.iYpFast[irow][Y] = -itmp16;
	thisGyro.iYpFast[irow][Z] = -thisGyro.iYpFast[irow][Z];
#endif // NED
#if THISCOORDSYSTEM == ANDROID
	thisGyro.iYpFast[irow][X] = -thisGyro.iYpFast[irow][X];
	thisGyro.iYpFast[irow][Y] = -thisGyro.iYpFast[irow][Y];
#endif // Android
#if THISCOORDSYSTEM == WIN8
	thisGyro.iYpFast[irow][X] = -thisGyro.iYpFast[irow][X];
	thisGyro.iYpFast[irow][Y] = -thisGyro.iYpFast[irow][Y];
#endif // Win8

	return;
}
