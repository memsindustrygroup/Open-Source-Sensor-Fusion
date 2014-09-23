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
// This is the file that contains the fusion routines.  It is STRONGLY RECOMMENDED
// that the casual developer NOT TOUCH THIS FILE.  The mathematics behind this file
// is extremely complex, and it will be very easy (almost inevitable) that you screw
// it up.
//
#include "Events.h"
#include "include_all.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "time.h"
#include "string.h"

void fInit_1DOF_P_BASIC(struct SV_1DOF_P_BASIC *pthisSV, float flpftimesecs, int16 iSensorFS, int16 iOverSampleRatio)
{
	// set algorithm sampling interval (typically 25Hz) and low pass filter
	pthisSV->fdeltat = (float) iOverSampleRatio / (float) iSensorFS;
	pthisSV->flpf = pthisSV->fdeltat / flpftimesecs;
	if (pthisSV->flpf > 1.0F)
	{
		pthisSV->flpf = 1.0F;
	}

	// clear the reset flag
	pthisSV->resetflag = false;

	return;
} // end fInit_1DOF_P_BASIC

void fInit_3DOF_G_BASIC(struct SV_3DOF_G_BASIC *pthisSV, float flpftimesecs, int16 iSensorFS, int16 iOverSampleRatio)
{
	// set algorithm sampling interval (typically 25Hz)
	pthisSV->fdeltat = (float) iOverSampleRatio / (float) iSensorFS;

	// set low pass filter constant with maximum value 1.0 (all pass) decreasing to 0.0 (increasing low pass)
	if (flpftimesecs > pthisSV->fdeltat)
		pthisSV->flpf = pthisSV->fdeltat / flpftimesecs;
	else
		pthisSV->flpf = 1.0F;

	// update the default quaternion type supported to the most sophisticated
	if (globals.DefaultQuaternionPacketType < Q3)
		globals.QuaternionPacketType = globals.DefaultQuaternionPacketType = Q3; 

	// clear the reset flag
	pthisSV->resetflag = false;

	return;
} // end fInit_3DOF_G_BASIC

void fInit_3DOF_B_BASIC(struct SV_3DOF_B_BASIC *pthisSV, float flpftimesecs, int16 iSensorFS, int16 iOverSampleRatio)
{
	// set algorithm sampling interval (typically 25Hz)
	pthisSV->fdeltat = (float) iOverSampleRatio / (float) iSensorFS;

	// set low pass filter constant with maximum value 1.0 (all pass) decreasing to 0.0 (increasing low pass)
	if (flpftimesecs > pthisSV->fdeltat)
		pthisSV->flpf = pthisSV->fdeltat / flpftimesecs;
	else
		pthisSV->flpf = 1.0F;

	// update the default quaternion type supported to the most sophisticated
	if (globals.DefaultQuaternionPacketType < Q3M)
		globals.QuaternionPacketType = globals.DefaultQuaternionPacketType = Q3M; 

	// clear the reset flag
	pthisSV->resetflag = false;

	return;
} // end fInit_3DOF_B_BASIC

void fInit_3DOF_Y_BASIC(struct SV_3DOF_Y_BASIC *pthisSV, int16 iSensorFS, int16 iOverSampleRatio)
{
	// compute the sampling time intervals (secs)
	pthisSV->fFastdeltat = 1.0F / (float) iSensorFS;
	pthisSV->fdeltat = (float) iOverSampleRatio * pthisSV->fFastdeltat;

	// initialize orientation estimates
	f3x3matrixAeqI(pthisSV->fR);
	fqAeq1(&(pthisSV->fq));

	// update the default quaternion type supported to the most sophisticated
	if (globals.DefaultQuaternionPacketType < Q3G)
		globals.QuaternionPacketType = globals.DefaultQuaternionPacketType = Q3G; 

	// clear the reset flag
	pthisSV->resetflag = false;

	return;
} // end fInit_3DOF_Y_BASIC

void fInit_6DOF_GB_BASIC(struct SV_6DOF_GB_BASIC *pthisSV, float flpftimesecs, int16 iSensorFS, int16 iOverSampleRatio)
{
	// set algorithm sampling interval (typically 25Hz)
	pthisSV->fdeltat = (float) iOverSampleRatio / (float) iSensorFS;

	// set low pass filter constant with maximum value 1.0 (all pass) decreasing to 0.0 (increasing low pass)
	if (flpftimesecs > pthisSV->fdeltat)
		pthisSV->flpf = pthisSV->fdeltat / flpftimesecs;
	else
		pthisSV->flpf = 1.0F;

	// update the default quaternion type supported to the most sophisticated
	if (globals.DefaultQuaternionPacketType < Q6MA)
		globals.QuaternionPacketType = globals.DefaultQuaternionPacketType = Q6MA; 

	// clear the reset flag
	pthisSV->resetflag = false;

	return;
} // end fInit_6DOF_GB_BASIC

// function initalizes the 6DOF accel + gyro Kalman filter algorithm
void fInit_6DOF_GY_KALMAN(struct SV_6DOF_GY_KALMAN *pthisSV, int16 iSensorFS, int16 iOverSampleRatio)
{
	int8 i, j;				// loop counters

	// reset the flag denoting that a first 6DOF orientation lock has been achieved
	pthisSV->iFirstOrientationLock = 0;

	// compute and store useful product terms to save floating point calculations later
	pthisSV->fFastdeltat = 1.0F / (float) iSensorFS;
	pthisSV->fdeltat = (float) iOverSampleRatio * pthisSV->fFastdeltat;
	pthisSV->fdeltatsq = pthisSV->fdeltat * pthisSV->fdeltat;
	pthisSV->fcasq = FCA_6DOF_GY_KALMAN * FCA_6DOF_GY_KALMAN;
	pthisSV->fQwbplusQvG = FQWB_6DOF_GY_KALMAN + FQVG_6DOF_GY_KALMAN;

	// initialize the fixed entries in the measurement matrix C
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 9; j++)
		{
			pthisSV->fC3x9[i][j]= 0.0F;
		}
	}
	pthisSV->fC3x9[0][6] = pthisSV->fC3x9[1][7] = pthisSV->fC3x9[2][8] = 1.0F;

	// zero a posteriori orientation, error vector xe+ (thetae+, be+, ae+) and b+
	f3x3matrixAeqI(pthisSV->fRPl);
	fqAeq1(&(pthisSV->fqPl));
	for (i = X; i <= Z; i++)
	{
		pthisSV->fThErrPl[i] = pthisSV->fbErrPl[i] = pthisSV->faErrSePl[i] = pthisSV->fbPl[i] = 0.0F;
	}

	// initialize noise variance for Qv and Qw matrix updates
	pthisSV->fQvAA = FQVA_6DOF_GY_KALMAN + FQWA_6DOF_GY_KALMAN + 
			FDEGTORAD * FDEGTORAD * pthisSV->fdeltatsq * (FQWB_6DOF_GY_KALMAN + FQVG_6DOF_GY_KALMAN);

	// initialize the 6x6 noise covariance matrix Qw of the a priori error vector xe-
	// Qw is then recursively updated as P+ = (1 - K * C) * P- = (1 - K * C) * Qw  and Qw updated from P+
	// zero the matrix Qw
	for (i = 0; i < 9; i++)
	{
		for (j = 0; j < 9; j++)
		{
			pthisSV->fQw9x9[i][j] = 0.0F;
		}
	}
	// loop over non-zero values
	for (i = 0; i < 3; i++)
	{
		// theta_e * theta_e terms
		pthisSV->fQw9x9[i][i] = FQWINITTHTH_6DOF_GY_KALMAN;
		// b_e * b_e terms
		pthisSV->fQw9x9[i + 3][i + 3] = FQWINITBB_6DOF_GY_KALMAN;
		// th_e * b_e terms
		pthisSV->fQw9x9[i][i + 3] = pthisSV->fQw9x9[i + 3][i] = FQWINITTHB_6DOF_GY_KALMAN;
		// a_e * a_e terms
		pthisSV->fQw9x9[i + 6][i + 6] = FQWINITAA_6DOF_GY_KALMAN;
	}

	// update the default quaternion type supported to the most sophisticated
	if (globals.DefaultQuaternionPacketType < Q6AG)
		globals.QuaternionPacketType = globals.DefaultQuaternionPacketType = Q6AG; 

	// clear the reset flag
	pthisSV->resetflag = false;

	return;
} // end fInit_6DOF_GY_KALMAN

// function initializes the 9DOF Kalman filter
void fInit_9DOF_GBY_KALMAN(struct SV_9DOF_GBY_KALMAN *pthisSV, int16 ithisCoordSystem, int16 iSensorFS, int16 iOverSampleRatio)
{
	int8 i, j;				// loop counters

	// reset the flag denoting that a first 9DOF orientation lock has been achieved
	pthisSV->iFirstOrientationLock = 0;

	// compute and store useful product terms to save floating point calculations later
	pthisSV->fFastdeltat = 1.0F / (float) iSensorFS;
	pthisSV->fdeltat = (float) iOverSampleRatio * pthisSV->fFastdeltat;
	pthisSV->fdeltatsq = pthisSV->fdeltat * pthisSV->fdeltat;
	pthisSV->fcasq = FCA_9DOF_GBY_KALMAN * FCA_9DOF_GBY_KALMAN;
	pthisSV->fcdsq = FCD_9DOF_GBY_KALMAN * FCD_9DOF_GBY_KALMAN;
	pthisSV->fQwbplusQvG = FQWB_9DOF_GBY_KALMAN + FQVG_9DOF_GBY_KALMAN;

	// initialize the fixed entries in the measurement matrix C
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 12; j++)
		{
			pthisSV->fC6x12[i][j]= 0.0F;
		}
	}
	pthisSV->fC6x12[0][6] = pthisSV->fC6x12[1][7] = pthisSV->fC6x12[2][8] = 1.0F; 
	pthisSV->fC6x12[3][9] = pthisSV->fC6x12[4][10] = pthisSV->fC6x12[5][11] = -1.0F; 

	// zero a posteriori orientation, error vector xe+ (thetae+, be+, de+, ae+) and b+ and inertial
	f3x3matrixAeqI(pthisSV->fRPl);
	fqAeq1(&(pthisSV->fqPl));
	for (i = X; i <= Z; i++)
	{
		pthisSV->fThErrPl[i] = pthisSV->fbErrPl[i] = pthisSV->faErrSePl[i] = pthisSV->fdErrSePl[i] = pthisSV->fbPl[i] = 0.0F;
	}

	// initialize the reference geomagnetic vector (uT, global frame)
	pthisSV->fDeltaPl = 0.0F;
	if (ithisCoordSystem == NED)
	{
		// initialize NED geomagnetic vector to zero degrees inclination
		pthisSV->fmGl[X] = DEFAULTB;
		pthisSV->fmGl[Y] = 0.0F;
		pthisSV->fmGl[Z] = 0.0F;
	}
	else
	{
		// initialize Android and Win8 geomagnetic vector to zero degrees inclination
		pthisSV->fmGl[X] = 0.0F;
		pthisSV->fmGl[Y] = DEFAULTB;
		pthisSV->fmGl[Z] = 0.0F;
	}

	// initialize noise variances for Qv and Qw matrix updates
	pthisSV->fQvAA = FQVA_9DOF_GBY_KALMAN + FQWA_9DOF_GBY_KALMAN + FDEGTORAD * FDEGTORAD * pthisSV->fdeltatsq * (FQWB_9DOF_GBY_KALMAN + FQVG_9DOF_GBY_KALMAN);
	pthisSV->fQvMM = FQVM_9DOF_GBY_KALMAN + FQWD_9DOF_GBY_KALMAN + FDEGTORAD * FDEGTORAD * pthisSV->fdeltatsq * DEFAULTB * DEFAULTB * (FQWB_9DOF_GBY_KALMAN + FQVG_9DOF_GBY_KALMAN);

	// initialize the 12x12 noise covariance matrix Qw of the a priori error vector xe-
	// Qw is then recursively updated as P+ = (1 - K * C) * P- = (1 - K * C) * Qw  and Qw updated from P+
	// zero the matrix Qw
	for (i = 0; i < 12; i++)
	{
		for (j = 0; j < 12; j++)
		{
			pthisSV->fQw12x12[i][j] = 0.0F;
		}
	}
	// loop over non-zero values
	for (i = 0; i < 3; i++)
	{
		// theta_e * theta_e terms
		pthisSV->fQw12x12[i][i] = FQWINITTHTH_9DOF_GBY_KALMAN;
		// b_e * b_e terms
		pthisSV->fQw12x12[i + 3][i + 3] = FQWINITBB_9DOF_GBY_KALMAN;
		// th_e * b_e terms
		pthisSV->fQw12x12[i][i + 3] = pthisSV->fQw12x12[i + 3][i] = FQWINITTHB_9DOF_GBY_KALMAN;
		// a_e * a_e terms
		pthisSV->fQw12x12[i + 6][i + 6] = FQWINITAA_9DOF_GBY_KALMAN;
		// d_e * d_e terms
		pthisSV->fQw12x12[i + 9][i + 9] = FQWINITDD_9DOF_GBY_KALMAN;
	}

	// update the default quaternion type supported to the most sophisticated
	if (globals.DefaultQuaternionPacketType < Q9)
		globals.QuaternionPacketType = globals.DefaultQuaternionPacketType = Q9; 

	// clear the reset flag
	pthisSV->resetflag = false;

	return;
} // end fInit_9DOF_GBY_KALMAN

// 1DOF pressure function
void fRun_1DOF_P_BASIC(struct SV_1DOF_P_BASIC *pthisSV, struct PressureSensor *pthisPressure, int32 loopcounter)
{
	// do a reset and return if requested
	if (pthisSV->resetflag)
	{
		fInit_1DOF_P_BASIC(pthisSV, 1.5F, SENSORFS, OVERSAMPLE_RATIO);
		return;
	}

	// low pass filter the block averaged (typically 25Hz) height and temperature readings
	fLPFScalar(&(pthisPressure->fHp), &(pthisSV->fLPHp), pthisSV->flpf, loopcounter);
	fLPFScalar(&(pthisPressure->fTp), &(pthisSV->fLPTp), pthisSV->flpf, loopcounter);

	return;
} // end fRun_1DOF_P_BASIC

// 3DOF orientation function which calls tilt functions and implements low pass filters
void fRun_3DOF_G_BASIC(struct SV_3DOF_G_BASIC *pthisSV, struct AccelSensor *pthisAccel, int32 loopcounter, int16 ithisCoordSystem)
{
	// do a reset and return if requested
	if (pthisSV->resetflag)
	{
		fInit_3DOF_G_BASIC(pthisSV, 0.2F, SENSORFS, OVERSAMPLE_RATIO);
		return;
	}

	// apply the tilt estimation algorithm to get the instantaneous orientation matrix
	if (ithisCoordSystem == NED)
	{
		// call NED tilt function
		f3DOFTiltNED(pthisSV->fR, pthisAccel->fGp);
	}
	else if (ithisCoordSystem == ANDROID)
	{
		// call Android tilt function
		f3DOFTiltAndroid(pthisSV->fR, pthisAccel->fGp);
	}
	else
	{
		// call Windows 8 tilt function
		f3DOFTiltWin8(pthisSV->fR, pthisAccel->fGp);
	}

	// compute the instanteneous quaternion from the instantaneous rotation matrix
	fQuaternionFromRotationMatrix(pthisSV->fR, &(pthisSV->fq));

	// low pass filter the orientation quaternion 
	fLPFOrientationQuaternion(&(pthisSV->fq), &(pthisSV->fLPq), pthisSV->flpf, pthisSV->fdeltat, pthisSV->fOmega, loopcounter);

	// compute the low pass rotation matrix and rotation vector
	fRotationMatrixFromQuaternion(pthisSV->fLPR, &(pthisSV->fLPq));
	fRotationVectorDegFromQuaternion(&(pthisSV->fLPq), pthisSV->fLPRVec);

	// calculate the Euler angles from the low pass orientation matrix
	if (ithisCoordSystem == NED)
	{
		// calculate NED Euler angles
		fNEDAnglesDegFromRotationMatrix(pthisSV->fLPR, &(pthisSV->fLPPhi), &(pthisSV->fLPThe), &(pthisSV->fLPPsi),
				&(pthisSV->fLPRho), &(pthisSV->fLPChi));
	}
	else if (ithisCoordSystem == ANDROID)
	{
		// calculate Android Euler angles
		fAndroidAnglesDegFromRotationMatrix(pthisSV->fLPR, &(pthisSV->fLPPhi), &(pthisSV->fLPThe), &(pthisSV->fLPPsi),
				&(pthisSV->fLPRho), &(pthisSV->fLPChi));
	}
	else
	{
		// calculate Windows 8 Euler angles
		fWin8AnglesDegFromRotationMatrix(pthisSV->fLPR, &(pthisSV->fLPPhi), &(pthisSV->fLPThe), &(pthisSV->fLPPsi),
				&(pthisSV->fLPRho), &(pthisSV->fLPChi));
	}

	// force the yaw and compass angles to zero
	pthisSV->fLPPsi = pthisSV->fLPRho = 0.0F;

	return;
} // end fRun_3DOF_G_BASIC

// 2D automobile eCompass
void fRun_3DOF_B_BASIC(struct SV_3DOF_B_BASIC *pthisSV, struct MagSensor *pthisMag, int32 loopcounter, int16 ithisCoordSystem)
{
	// do a reset and return if requested
	if (pthisSV->resetflag)
	{
		fInit_3DOF_B_BASIC(pthisSV, 0.6F, SENSORFS, OVERSAMPLE_RATIO);
		return;
	}

	// calculate the 3DOF magnetometer orientation matrix from fBc
	if (ithisCoordSystem == NED)
	{
		// call NED magnetic rotation matrix function
		f3DOFMagnetometerMatrixNED(pthisSV->fR, pthisMag->fBc);
	}
	else if (ithisCoordSystem == ANDROID)
	{
		// call Android magnetic rotation matrix function
		f3DOFMagnetometerMatrixAndroid(pthisSV->fR, pthisMag->fBc);
	}
	else
	{
		// call Windows 8 magnetic rotation matrix function
		f3DOFMagnetometerMatrixWin8(pthisSV->fR, pthisMag->fBc);
	}

	// compute the instanteneous quaternion from the instantaneous rotation matrix
	fQuaternionFromRotationMatrix(pthisSV->fR, &(pthisSV->fq));

	// low pass filter the orientation quaternion and compute the low pass rotation matrix
	fLPFOrientationQuaternion(&(pthisSV->fq), &(pthisSV->fLPq), pthisSV->flpf, pthisSV->fdeltat, pthisSV->fOmega, loopcounter);
	fRotationMatrixFromQuaternion(pthisSV->fLPR, &(pthisSV->fLPq));
	fRotationVectorDegFromQuaternion(&(pthisSV->fLPq), pthisSV->fLPRVec);

	// calculate the Euler angles from the low pass orientation matrix
	if (ithisCoordSystem == NED)
	{
		// calculate NED Euler angles
		fNEDAnglesDegFromRotationMatrix(pthisSV->fLPR, &(pthisSV->fLPPhi), &(pthisSV->fLPThe), &(pthisSV->fLPPsi),
				&(pthisSV->fLPRho), &(pthisSV->fLPChi));
	}
	else if (ithisCoordSystem == ANDROID)
	{
		// calculate Android Euler angles
		fAndroidAnglesDegFromRotationMatrix(pthisSV->fLPR, &(pthisSV->fLPPhi), &(pthisSV->fLPThe), &(pthisSV->fLPPsi),
				&(pthisSV->fLPRho), &(pthisSV->fLPChi));
	}
	else
	{
		// calculate Windows 8 Euler angles
		fWin8AnglesDegFromRotationMatrix(pthisSV->fLPR, &(pthisSV->fLPPhi), &(pthisSV->fLPThe), &(pthisSV->fLPPsi),
				&(pthisSV->fLPRho), &(pthisSV->fLPChi));
	}

	return;
}

// basic gyro integration function
void fRun_3DOF_Y_BASIC(struct SV_3DOF_Y_BASIC *pthisSV, struct GyroSensor *pthisGyro, int32 loopcounter, int16 ithisCoordSystem, int16 iOverSampleRatio)
{
	float rvec[3];							// rotation vector
	int8 i, j;								// loop counters

	// do a reset and return if requested
	if (pthisSV->resetflag)
	{
		fInit_3DOF_Y_BASIC(pthisSV, SENSORFS, OVERSAMPLE_RATIO);
		return;
	}

	// set the angular velocity to the raw gyro reading: omega[k] = yG[k]
	for (i = X; i <= Z; i++)
	{
		pthisSV->fOmega[i] = pthisGyro->fYp[i];
	}

	// integrate the buffered high frequency (typically 200Hz) gyro readings
	for (j = 0; j < iOverSampleRatio; j++)
	{
		// compute the incremental fast (typically 200Hz) rotation vector rvec (deg)
		for (i = X; i <= Z; i++)
		{
			rvec[i] = ((float)pthisGyro->iYpFast[j][i] * pthisGyro->fDegPerSecPerCount) * pthisSV->fFastdeltat;
		}

		// compute the incremental quaternion fDeltaq from the rotation vector
		fQuaternionFromRotationVectorDeg(&(pthisSV->fDeltaq), rvec, 1.0F);

		// incrementally rotate the orientation quaternion fq
		qAeqAxB(&(pthisSV->fq), &(pthisSV->fDeltaq));
	}

	// re-normalize the orientation quaternion to stop error propagation
	// the renormalization function ensures that the scalar component q0 is non-negative
	fqAeqNormqA(&(pthisSV->fq));

	// get the rotation matrix from the quaternion
	fRotationMatrixFromQuaternion(pthisSV->fR, &(pthisSV->fq));

	// compute the rotation vector from the a posteriori quaternion
	fRotationVectorDegFromQuaternion(&(pthisSV->fq), pthisSV->fRVec);

	// *********************************************************************************
	// compute the Euler angles from the orientation matrix
	// *********************************************************************************

	if (ithisCoordSystem == NED)
	{
		// calculate the NED Euler angles
		fNEDAnglesDegFromRotationMatrix(pthisSV->fR, &(pthisSV->fPhi), &(pthisSV->fThe), &(pthisSV->fPsi),
				&(pthisSV->fRho), &(pthisSV->fChi));
	}
	else if (ithisCoordSystem == ANDROID)
	{
		// calculate the Android Euler angles
		fAndroidAnglesDegFromRotationMatrix(pthisSV->fR, &(pthisSV->fPhi), &(pthisSV->fThe), &(pthisSV->fPsi),
				&(pthisSV->fRho), &(pthisSV->fChi));
	}
	else
	{
		// calculate Win8 Euler angles
		fWin8AnglesDegFromRotationMatrix(pthisSV->fR, &(pthisSV->fPhi), &(pthisSV->fThe), &(pthisSV->fPsi),
				&(pthisSV->fRho), &(pthisSV->fChi));
	}

	return;
} // end fRun_3DOF_Y_BASIC

// 6DOF orientation function which calls ecompass and implements low pass filters
void fRun_6DOF_GB_BASIC(struct SV_6DOF_GB_BASIC *pthisSV, struct MagSensor *pthisMag, struct AccelSensor *pthisAccel, int32 loopcounter, int16 ithisCoordSystem)
{
	// do a reset and return if requested
	if (pthisSV->resetflag)
	{
		fInit_6DOF_GB_BASIC(pthisSV, 0.6F, SENSORFS, OVERSAMPLE_RATIO);
		return;
	}

	// call the eCompass algorithm to get the instantaneous orientation matrix and inclination angle
	if (ithisCoordSystem == NED)
	{
		// call the NED eCompass
		feCompassNED(pthisSV->fR, &(pthisSV->fDelta), pthisMag->fBc, pthisAccel->fGp);
	}
	else if  (ithisCoordSystem == ANDROID)
	{
		// call the Android eCompass
		feCompassAndroid(pthisSV->fR, &(pthisSV->fDelta), pthisMag->fBc, pthisAccel->fGp);
	}
	else
	{
		// call the Win8 eCompass
		feCompassWin8(pthisSV->fR, &(pthisSV->fDelta), pthisMag->fBc, pthisAccel->fGp);
	}

	// compute the instanteneous quaternion from the instantaneous rotation matrix
	fQuaternionFromRotationMatrix(pthisSV->fR, &(pthisSV->fq));

	// low pass filter the orientation quaternion and compute the low pass rotation matrix
	fLPFOrientationQuaternion(&(pthisSV->fq), &(pthisSV->fLPq), pthisSV->flpf, pthisSV->fdeltat, pthisSV->fOmega, loopcounter);
	fRotationMatrixFromQuaternion(pthisSV->fLPR, &(pthisSV->fLPq));
	fRotationVectorDegFromQuaternion(&(pthisSV->fLPq), pthisSV->fLPRVec);

	// compute the low pass filtered Euler angles
	if (ithisCoordSystem == NED)
	{
		// calculate the NED Euler angles
		fNEDAnglesDegFromRotationMatrix(pthisSV->fLPR, &(pthisSV->fLPPhi), &(pthisSV->fLPThe), &(pthisSV->fLPPsi),
				&(pthisSV->fLPRho), &(pthisSV->fLPChi));
	}
	else if (ithisCoordSystem == ANDROID)
	{
		// calculate the Android Euler angles
		fAndroidAnglesDegFromRotationMatrix(pthisSV->fLPR, &(pthisSV->fLPPhi), &(pthisSV->fLPThe), &(pthisSV->fLPPsi),
				&(pthisSV->fLPRho), &(pthisSV->fLPChi));
	}
	else
	{
		// calculate the Windows 8 Euler angles
		fWin8AnglesDegFromRotationMatrix(pthisSV->fLPR, &(pthisSV->fLPPhi), &(pthisSV->fLPThe), &(pthisSV->fLPPsi),
				&(pthisSV->fLPRho), &(pthisSV->fLPChi));
	}

	// low pass filter the geomagnetic inclination angle with a simple exponential filter
	fLPFScalar(&(pthisSV->fDelta), &(pthisSV->fLPDelta), pthisSV->flpf, loopcounter);

	return;
}  // end fRun_6DOF_GB_BASIC

// 6DOF accel + gyro Kalman filter algorithm 
void fRun_6DOF_GY_KALMAN(struct SV_6DOF_GY_KALMAN *pthisSV, struct AccelSensor *pthisAccel, struct GyroSensor *pthisGyro,
		int16 ithisCoordSystem, int16 iOverSampleRatio)
{	
	// local arrays and scalars
	float rvec[3];							// rotation vector
	float ftmpA9x3[9][3];					// scratch array

	// assorted array pointers
	float *pfPPlus9x9kj;
	float *pfPPlus9x9ij;
	float *pfK9x3ij;
	float *pfK9x3ik;
	float *pftmpA9x3ik;
	float *pftmpA9x3ij;
	float *pftmpA9x3kj;
	float *pfQw9x9ij;
	float *pfQw9x9ik;
	float *pfQw9x9kj;
	float *pfC3x9ik;
	float *pfC3x9jk;

	int8 i, j, k;							// loop counters

	// working arrays for 3x3 matrix inversion
	float *pfRows[3];
	int8 iColInd[3];
	int8 iRowInd[3];
	int8 iPivot[3];

	// do a reset and return if requested
	if (pthisSV->resetflag)
	{
		fInit_6DOF_GY_KALMAN(pthisSV, SENSORFS, OVERSAMPLE_RATIO);
		return;
	}

	// do a once-only orientation lock to accelerometer tilt 
	if (!pthisSV->iFirstOrientationLock)
	{
		// get the 3DOF orientation matrix and initial inclination angle
		if (ithisCoordSystem == NED)
		{
			// call NED tilt function
			f3DOFTiltNED(pthisSV->fRPl, pthisAccel->fGpFast);
		}
		else if (ithisCoordSystem == ANDROID)
		{
			// call Android tilt function
			f3DOFTiltAndroid(pthisSV->fRPl, pthisAccel->fGpFast);
		}
		else
		{
			// call Windows 8 tilt function
			f3DOFTiltWin8(pthisSV->fRPl, pthisAccel->fGpFast);
		}

		// get the orientation quaternion from the orientation matrix
		fQuaternionFromRotationMatrix(pthisSV->fRPl, &(pthisSV->fqPl));

		// set the orientation lock flag so this initial alignment is only performed once
		pthisSV->iFirstOrientationLock = 1;
	}

	// *********************************************************************************
	// calculate a priori rotation matrix
	// *********************************************************************************

	// compute the angular velocity from the average of the high frequency gyro readings.
	// omega[k] = yG[k] - b-[k] = yG[k] - b+[k-1] (deg/s)
	// this involves a small angle approximation but the resulting angular velocity is
	// only computed for transmission over bluetooth and not used for orientation determination.
	for (i = X; i <= Z; i++)
	{
		pthisSV->fOmega[i] = pthisGyro->fYp[i] - pthisSV->fbPl[i];
	}

	// initialize the a priori orientation quaternion to the a posteriori orientation estimate
	pthisSV->fqMi = pthisSV->fqPl;

	// integrate the buffered high frequency (typically 200Hz) gyro readings
	for (j = 0; j < iOverSampleRatio; j++)
	{
		// compute the incremental fast (typically 200Hz) rotation vector rvec (deg)
		for (i = X; i <= Z; i++)
		{
			rvec[i] = (((float)pthisGyro->iYpFast[j][i] * pthisGyro->fDegPerSecPerCount) - pthisSV->fbPl[i]) * pthisSV->fFastdeltat;
		}

		// compute the incremental quaternion fDeltaq from the rotation vector
		fQuaternionFromRotationVectorDeg(&(pthisSV->fDeltaq), rvec, 1.0F);

		// incrementally rotate the a priori orientation quaternion fqMi
		// the a posteriori orientation is re-normalized later so this update is stable
		qAeqAxB(&(pthisSV->fqMi), &(pthisSV->fDeltaq));
	}

	// get the a priori rotation matrix from the a priori quaternion
	fRotationMatrixFromQuaternion(pthisSV->fRMi, &(pthisSV->fqMi));

	// *********************************************************************************
	// calculate a priori gyro and accelerometer estimates of the gravity vector
	// and the error between the two
	// *********************************************************************************

	// compute the a priori **gyro** estimate of the gravitational vector (g, sensor frame)
	// using an absolute rotation of the global frame gravity vector (with magnitude 1g)
	for (i = X; i <= Z; i++)
	{
		if (ithisCoordSystem == NED)
		{
			// NED gravity is along positive z axis
			pthisSV->fgSeGyMi[i] = pthisSV->fRMi[i][Z];
		}
		else
		{
			// Android and Win8 gravity are along negative z axis
			pthisSV->fgSeGyMi[i] = -pthisSV->fRMi[i][Z];
		}

		// compute a priori acceleration (a-) (g, sensor frame) from a posteriori estimate (g, sensor frame)
		pthisSV->faSeMi[i] = FCA_6DOF_GY_KALMAN * pthisSV->faSePl[i];

		// compute the a priori gravity error vector (accelerometer minus gyro estimates) (g, sensor frame)
		if ((ithisCoordSystem == NED) || (ithisCoordSystem == WIN8))
		{
			// NED and Windows 8 have positive sign for gravity: y = g - a and g = y + a
			pthisSV->fgErrSeMi[i] = pthisAccel->fGpFast[i] + pthisSV->faSeMi[i] - pthisSV->fgSeGyMi[i];
		}
		else
		{
			// Android has negative sign for gravity: y = a - g, g = -y + a
			pthisSV->fgErrSeMi[i] = -pthisAccel->fGpFast[i] + pthisSV->faSeMi[i] - pthisSV->fgSeGyMi[i];
		}
	}

	// *********************************************************************************
	// update variable elements of measurement matrix C
	// *********************************************************************************

	// update measurement matrix C (3x9) with -alpha(g-)x from gyro (g, sensor frame)
	pthisSV->fC3x9[0][1] = FDEGTORAD * pthisSV->fgSeGyMi[Z];
	pthisSV->fC3x9[0][2] = -FDEGTORAD * pthisSV->fgSeGyMi[Y];
	pthisSV->fC3x9[1][2] = FDEGTORAD * pthisSV->fgSeGyMi[X];
	pthisSV->fC3x9[1][0] = -pthisSV->fC3x9[0][1];
	pthisSV->fC3x9[2][0] = -pthisSV->fC3x9[0][2];
	pthisSV->fC3x9[2][1] = -pthisSV->fC3x9[1][2];
	pthisSV->fC3x9[0][4] = -pthisSV->fdeltat * pthisSV->fC3x9[0][1];
	pthisSV->fC3x9[0][5] = -pthisSV->fdeltat * pthisSV->fC3x9[0][2];
	pthisSV->fC3x9[1][5] = -pthisSV->fdeltat * pthisSV->fC3x9[1][2];
	pthisSV->fC3x9[1][3]= -pthisSV->fC3x9[0][4];
	pthisSV->fC3x9[2][3]= -pthisSV->fC3x9[0][5];
	pthisSV->fC3x9[2][4]= -pthisSV->fC3x9[1][5];

	// *********************************************************************************
	// calculate the Kalman gain matrix K (9x3)
	// K = P- * C^T * inv(C * P- * C^T + Qv) = Qw * C^T * inv(C * Qw * C^T + Qv)
	// Qw is used as a proxy for P- throughout the code
	// P+ is used here as a working array to reduce RAM usage and is re-computed later
	// *********************************************************************************

	// set ftmpA9x3 = P- * C^T = Qw * C^T where Qw and C are both sparse
	// C also has a significant number of +1 and -1 entries
	// ftmpA9x3 is also sparse but not symmetric
	for (i = 0; i < 9; i++) // loop over rows of ftmpA9x3
	{
		// initialize pftmpA9x3ij for current i, j=0
		pftmpA9x3ij = ftmpA9x3[i];

		for (j = 0; j < 3; j++) // loop over columns of ftmpA9x3
		{
			// zero ftmpA9x3[i][j]
			*pftmpA9x3ij = 0.0F;

			// initialize pfC3x9jk for current j, k=0
			pfC3x9jk = pthisSV->fC3x9[j];

			// initialize pfQw9x9ik for current i, k=0
			pfQw9x9ik = pthisSV->fQw9x9[i];

			// sum matrix products over inner loop over k
			for (k = 0; k < 9; k++)
			{
				if ((*pfQw9x9ik != 0.0F) && (*pfC3x9jk != 0.0F))
				{
					if (*pfC3x9jk == 1.0F)
						*pftmpA9x3ij += *pfQw9x9ik;
					else if (*pfC3x9jk == -1.0F)
						*pftmpA9x3ij -= *pfQw9x9ik;
					else
						*pftmpA9x3ij += *pfQw9x9ik * *pfC3x9jk;
				}

				// increment pfC3x9jk and pfQw9x9ik for next iteration of k
				pfC3x9jk++;
				pfQw9x9ik++;

			} // end of loop over k

			// increment pftmpA9x3ij for next iteration of j
			pftmpA9x3ij++;

		} // end of loop over j
	} // end of loop over i

	// set symmetric P+ (3x3 scratch sub-matrix) to C * P- * C^T + Qv
	// = C * (Qw * C^T) + Qv = C * ftmpA9x3 + Qv
	// both C and ftmpA9x3 are sparse but not symmetric
	for (i = 0; i < 3; i++) // loop over rows of P+
	{
		// initialize pfPPlus9x9ij for current i, j=i
		pfPPlus9x9ij = pthisSV->fPPlus9x9[i] + i;

		for (j = i; j < 3; j++) // loop over above diagonal columns of P+
		{
			// zero P+[i][j] 
			*pfPPlus9x9ij = 0.0F;

			// initialize pfC3x9ik for current i, k=0
			pfC3x9ik = pthisSV->fC3x9[i];

			// initialize pftmpA9x3kj for current j, k=0
			pftmpA9x3kj = *ftmpA9x3 + j;

			// sum matrix products over inner loop over k
			for (k = 0; k < 9; k++)
			{
				if ((*pfC3x9ik != 0.0F) && (*pftmpA9x3kj != 0.0F))
				{
					if (*pfC3x9ik == 1.0F)
						*pfPPlus9x9ij += *pftmpA9x3kj;
					else if (*pfC3x9ik == -1.0F)
						*pfPPlus9x9ij -= *pftmpA9x3kj;
					else
						*pfPPlus9x9ij += *pfC3x9ik * *pftmpA9x3kj;
				}

				// update pfC3x9ik and pftmpA9x3kj for next iteration of k
				pfC3x9ik++;						
				pftmpA9x3kj += 3;

			} // end of loop over k

			// increment pfPPlus9x9ij for next iteration of j
			pfPPlus9x9ij++;

		} // end of loop over j
	} // end of loop over i

	// add in noise covariance terms to the diagonal
	pthisSV->fPPlus9x9[0][0] +=  pthisSV->fQvAA;
	pthisSV->fPPlus9x9[1][1] +=  pthisSV->fQvAA;
	pthisSV->fPPlus9x9[2][2] +=  pthisSV->fQvAA;

	// copy above diagonal terms of P+ (3x3 scratch sub-matrix) to below diagonal terms
	pthisSV->fPPlus9x9[1][0] = pthisSV->fPPlus9x9[0][1];
	pthisSV->fPPlus9x9[2][0] = pthisSV->fPPlus9x9[0][2];
	pthisSV->fPPlus9x9[2][1] = pthisSV->fPPlus9x9[1][2];

	// calculate inverse of P+ (3x3 scratch sub-matrix) = inv(C * P- * C^T + Qv) = inv(C * Qw * C^T + Qv)
	for (i = 0; i < 3; i++)
	{
		pfRows[i] = pthisSV->fPPlus9x9[i];
	}
	fmatrixAeqInvA(pfRows, iColInd, iRowInd, iPivot, 3);

	// set K = P- * C^T * inv(C * P- * C^T + Qv) = Qw * C^T * inv(C * Qw * C^T + Qv)
	// = ftmpA9x3 * P+ (3x3 sub-matrix)
	// ftmpA9x3 = Qw * C^T is sparse but P+ (3x3 sub-matrix) is not
	// K is not symmetric because C is not symmetric
	for (i = 0; i < 9; i++) // loop over rows of K9x3
	{
		// initialize pfK9x3ij for i, j=0
		pfK9x3ij = pthisSV->fK9x3[i];

		for (j = 0; j < 3; j++) // loop over columns of K9x3
		{
			// zero the matrix element fK9x3[i][j]
			*pfK9x3ij = 0.0F;

			// initialize pftmpA9x3ik for i, k=0
			pftmpA9x3ik = ftmpA9x3[i];

			// initialize pfPPlus9x9kj for j, k=0
			pfPPlus9x9kj = *pthisSV->fPPlus9x9 + j;

			// sum matrix products over inner loop over k
			for (k = 0; k < 3; k++)
			{
				if (*pftmpA9x3ik != 0.0F)
				{
					*pfK9x3ij += *pftmpA9x3ik * *pfPPlus9x9kj;
				}

				// increment pftmpA9x3ik and pfPPlus9x9kj for next iteration of k
				pftmpA9x3ik++;
				pfPPlus9x9kj += 9;

			} // end of loop over k

			// increment pfK9x3ij for the next iteration of j
			pfK9x3ij++;

		} // end of loop over j
	} // end of loop over i

	// *********************************************************************************
	// calculate a posteriori error estimate: xe+ = K * ze-
	// *********************************************************************************

	// update the a posteriori state vector
	for (i = X; i <= Z; i++)
	{
		// zero a posteriori error terms
		pthisSV->fThErrPl[i] = pthisSV->fbErrPl[i] = pthisSV->faErrSePl[i] = 0.0F;

		// accumulate the error vector terms K * ze-
		for (k = 0; k < 3; k++)
		{
			pthisSV->fThErrPl[i] += pthisSV->fK9x3[i][k] * pthisSV->fgErrSeMi[k];
			pthisSV->fbErrPl[i] += pthisSV->fK9x3[i + 3][k] * pthisSV->fgErrSeMi[k];
			pthisSV->faErrSePl[i] += pthisSV->fK9x3[i + 6][k] * pthisSV->fgErrSeMi[k];
		}
	}

	// *********************************************************************************
	// apply the a posteriori error corrections to the a posteriori state vector
	// *********************************************************************************

	// get the a posteriori delta quaternion
	fQuaternionFromRotationVectorDeg(&(pthisSV->fDeltaq), pthisSV->fThErrPl, -1.0F);

	// compute the a posteriori orientation quaternion fqPl = fqMi * Deltaq(-thetae+)
	// the resulting quaternion may have negative scalar component q0
	qAeqBxC(&(pthisSV->fqPl), &(pthisSV->fqMi), &(pthisSV->fDeltaq));

	// normalize the a posteriori orientation quaternion to stop error propagation 
	// the renormalization function ensures that the scalar component q0 is non-negative
	fqAeqNormqA(&(pthisSV->fqPl));

	// compute the a posteriori rotation matrix from the a posteriori quaternion
	fRotationMatrixFromQuaternion(pthisSV->fRPl, &(pthisSV->fqPl));

	// compute the rotation vector from the a posteriori quaternion
	fRotationVectorDegFromQuaternion(&(pthisSV->fqPl), pthisSV->fRVecPl);

	// update the a posteriori gyro offset vector b+ and linear acceleration vector a+ (sensor frame)
	for (i = X; i <= Z; i++)
	{
		// b+[k] = b-[k] - be+[k] = b+[k] - be+[k] (deg/s)
		pthisSV->fbPl[i] -= pthisSV->fbErrPl[i];
		// a+ = a- - ae+ (g, sensor frame)
		pthisSV->faSePl[i] = pthisSV->faSeMi[i] - pthisSV->faErrSePl[i];
	}

	// *********************************************************************************
	// compute the a posteriori Euler angles from the orientation matrix
	// *********************************************************************************

	if (ithisCoordSystem == NED)
	{
		// calculate the NED Euler angles
		fNEDAnglesDegFromRotationMatrix(pthisSV->fRPl, &(pthisSV->fPhiPl), &(pthisSV->fThePl), &(pthisSV->fPsiPl),
				&(pthisSV->fRhoPl), &(pthisSV->fChiPl));
	}
	else if (ithisCoordSystem == ANDROID)
	{
		// calculate the Android Euler angles
		fAndroidAnglesDegFromRotationMatrix(pthisSV->fRPl, &(pthisSV->fPhiPl), &(pthisSV->fThePl), &(pthisSV->fPsiPl),
				&(pthisSV->fRhoPl), &(pthisSV->fChiPl));
	}
	else
	{
		// calculate Win8 Euler angles
		fWin8AnglesDegFromRotationMatrix(pthisSV->fRPl, &(pthisSV->fPhiPl), &(pthisSV->fThePl), &(pthisSV->fPsiPl),
				&(pthisSV->fRhoPl), &(pthisSV->fChiPl));
	}

	// ***********************************************************************************
	// calculate (symmetric) a posteriori error covariance matrix P+
	// P+ = (I12 - K * C) * P- = (I12 - K * C) * Qw = Qw - K * (C * Qw)
	// both Qw and P+ are used as working arrays in this section
	// at the end of this section, P+ is valid but Qw is over-written
	// ***********************************************************************************

	// set P+ (3x9 scratch sub-matrix) to the product C (3x9) * Qw (9x9)
	// where both C and Qw are sparse and C has a significant number of +1 entries
	// the resulting matrix is sparse but not symmetric
	for (i = 0; i < 3; i++) // loop over the rows of P+
	{
		// initialize pfPPlus9x9ij for current i, j=0
		pfPPlus9x9ij = pthisSV->fPPlus9x9[i];

		for (j = 0; j < 9; j++) // loop over the columns of P+
		{
			// zero P+[i][j] 
			*pfPPlus9x9ij = 0.0F;

			// initialize pfC3x9ik for current i, k=0
			pfC3x9ik = pthisSV->fC3x9[i];

			// initialize pfQw9x9kj for current j, k=0
			pfQw9x9kj = &pthisSV->fQw9x9[0][j];

			// sum matrix products over inner loop over k
			for (k = 0; k < 9; k++)
			{
				if ((*pfC3x9ik != 0.0F) && (*pfQw9x9kj != 0.0F))
				{
					if (*pfC3x9ik == 1.0F)
						*pfPPlus9x9ij += *pfQw9x9kj;
					else if (*pfC3x9ik == -1.0F)
						*pfPPlus9x9ij -= *pfQw9x9kj;
					else
						*pfPPlus9x9ij += *pfC3x9ik * *pfQw9x9kj;
				}

				// update pfC3x9ik and pfQw9x9kj for next iteration of k
				pfC3x9ik++;
				pfQw9x9kj += 9;

			} // end of loop over k

			// increment pfPPlus9x9ij for next iteration of j
			pfPPlus9x9ij++;

		} // end of loop over j
	} // end of loop over i

	// compute P+ = (I9 - K * C) * Qw = Qw - K * (C * Qw) = Qw - K * P+ (3x9 sub-matrix)
	// storing result P+ in Qw and over-writing Qw which is OK since Qw is later computed from P+
	// where working array P+ (6x12 sub-matrix) is sparse but K is not sparse
	// only on and above diagonal terms of P+ are computed since P+ is symmetric
	for (i = 0; i < 9; i++)
	{
		// initialize pfQw9x9ij for i, j=i
		pfQw9x9ij = pthisSV->fQw9x9[i] + i;

		for (j = i; j < 9; j++)
		{
			// initialize pfK9x3ik for i, k=0
			pfK9x3ik = pthisSV->fK9x3[i];

			// initialize pfPPlus9x9kj for j, k=0
			pfPPlus9x9kj = *pthisSV->fPPlus9x9 + j;

			// compute on and above diagonal matrix entry
			for (k = 0; k < 3; k++)
			{
				// check for non-zero values since P+ (3x9 scratch sub-matrix) is sparse
				if (*pfPPlus9x9kj != 0.0F)
				{
					*pfQw9x9ij -= *pfK9x3ik * *pfPPlus9x9kj;
				}

				// increment pfK9x3ik and pfPPlus9x9kj for next iteration of k
				pfK9x3ik++;
				pfPPlus9x9kj += 9;

			} // end of loop over k

			// increment pfQw9x9ij for next iteration of j
			pfQw9x9ij++;

		} // end of loop over j
	} // end of loop over i

	// Qw now holds the on and above diagonal elements of P+ (9x9)
	// so perform a simple copy to the all elements of P+
	// after execution of this code P+ is valid but Qw remains invalid
	for (i = 0; i < 9; i++)
	{
		// initialize pfPPlus9x9ij and pfQw9x9ij for i, j=i
		pfPPlus9x9ij = pthisSV->fPPlus9x9[i] + i;
		pfQw9x9ij = pthisSV->fQw9x9[i] + i;

		// copy the on-diagonal elements and increment pointers to enter loop at j=i+1
		*(pfPPlus9x9ij++) = *(pfQw9x9ij++);

		// loop over above diagonal columns j copying to below-diagonal elements
		for (j = i + 1; j < 9; j++)
		{
			*(pfPPlus9x9ij++)= pthisSV->fPPlus9x9[j][i] = *(pfQw9x9ij++);
		}
	}

	// *********************************************************************************
	// re-create the noise covariance matrix Qw=fn(P+) for the next iteration
	// using the elements of P+ which are now valid
	// Qw was over-written earlier but is here recomputed (all elements)
	// *********************************************************************************

	// zero the matrix Qw (9x9)
	for (i = 0; i < 9; i++)
	{
		for (j = 0; j < 9; j++)
		{
			pthisSV->fQw9x9[i][j] = 0.0F;
		}
	}

	// update the covariance matrix components
	for (i = 0; i < 3; i++)
	{
		// Qw[th-th-] = Qw[0-2][0-2] = E[th-(th-)^T] = Q[th+th+] + deltat^2 * (Q[b+b+] + (Qwb + QvG) * I)
		pthisSV->fQw9x9[i][i] = pthisSV->fPPlus9x9[i][i] + pthisSV->fdeltatsq * (pthisSV->fPPlus9x9[i + 3][i + 3] + pthisSV->fQwbplusQvG);

		// Qw[b-b-] = Qw[3-5][3-5] = E[b-(b-)^T] = Q[b+b+] + Qwb * I
		pthisSV->fQw9x9[i + 3][i + 3] = pthisSV->fPPlus9x9[i + 3][i + 3] + FQWB_6DOF_GY_KALMAN;

		// Qw[th-b-] = Qw[0-2][3-5] = E[th-(b-)^T] = -deltat * (Q[b+b+] + Qwb * I) = -deltat * Qw[b-b-]
		pthisSV->fQw9x9[i][i + 3] = pthisSV->fQw9x9[i + 3][i] = -pthisSV->fdeltat * pthisSV->fQw9x9[i + 3][i + 3];

		// Qw[a-a-] = Qw[6-8][6-8] = E[a-(a-)^T] = ca^2 * Q[a+a+] + Qwa * I
		pthisSV->fQw9x9[i + 6][i + 6] = pthisSV->fcasq * pthisSV->fPPlus9x9[i + 6][i + 6] + FQWA_6DOF_GY_KALMAN;
	}

	return;
} // end fRun_6DOF_GY_KALMAN

// 9DOF orientation function implemented using a 12 element Kalman filter
void fRun_9DOF_GBY_KALMAN(struct SV_9DOF_GBY_KALMAN *pthisSV, struct AccelSensor *pthisAccel, struct MagSensor *pthisMag, struct GyroSensor *pthisGyro,
		struct MagCalibration *pthisMagCal, int16 ithisCoordSystem, int16 iOverSampleRatio)
{	
	// local scalars and arrays
	float fopp, fadj, fhyp;						// opposite, adjacent and hypoteneuse
	float fsindelta, fcosdelta;					// sin and cos of inclination angle delta
	float rvec[3];								// rotation vector
	float ftmp;									// scratch variable
	float ftmpA12x6[12][6];						// scratch array
	int8 i, j, k;								// loop counters
	int8 iMagJamming;							// magnetic jamming flag

	// assorted array pointers
	float *pfPPlus12x12ij;
	float *pfPPlus12x12kj;
	float *pfQw12x12ij;
	float *pfQw12x12ik;
	float *pfQw12x12kj;
	float *pfK12x6ij;
	float *pfK12x6ik;
	float *pftmpA12x6ik;
	float *pftmpA12x6kj;
	float *pftmpA12x6ij;
	float *pfC6x12ik;
	float *pfC6x12jk;

	// working arrays for 6x6 matrix inversion
	float *pfRows[6];
	int8 iColInd[6];
	int8 iRowInd[6];
	int8 iPivot[6];

	// do a reset and return if requested
	if (pthisSV->resetflag)
	{
		fInit_9DOF_GBY_KALMAN(pthisSV, THISCOORDSYSTEM, SENSORFS, OVERSAMPLE_RATIO);
		return;
	}

	// *********************************************************************************
	// initial orientation lock to accelerometer and magnetometer eCompass orientation
	// *********************************************************************************

	// do a once-only orientation lock after the first valid magnetic calibration 
	if (pthisMagCal->iValidMagCal && !pthisSV->iFirstOrientationLock)
	{
		// get the 6DOF orientation matrix and initial inclination angle
		if (ithisCoordSystem == NED)
		{
			feCompassNED(pthisSV->fRPl, &(pthisSV->fDeltaPl), pthisMag->fBcFast, pthisAccel->fGpFast);
		}
		else if (ithisCoordSystem == ANDROID)
		{
			feCompassAndroid(pthisSV->fRPl, &(pthisSV->fDeltaPl), pthisMag->fBcFast, pthisAccel->fGpFast);
		}
		else if (ithisCoordSystem == WIN8)
		{
			feCompassWin8(pthisSV->fRPl, &(pthisSV->fDeltaPl), pthisMag->fBcFast, pthisAccel->fGpFast);
		}

		// get the orientation quaternion from the orientation matrix
		fQuaternionFromRotationMatrix(pthisSV->fRPl, &(pthisSV->fqPl));

		// set the orientation lock flag so this initial alignment is only performed once
		pthisSV->iFirstOrientationLock = 1;
	}

	// *********************************************************************************
	// calculate a priori rotation matrix
	// *********************************************************************************

	// compute the angular velocity from the averaged high frequency gyro reading.
	// omega[k] = yG[k] - b-[k] = yG[k] - b+[k-1] (deg/s)
	pthisSV->fOmega[X] = pthisGyro->fYp[X] - pthisSV->fbPl[X];
	pthisSV->fOmega[Y] = pthisGyro->fYp[Y] - pthisSV->fbPl[Y];
	pthisSV->fOmega[Z] = pthisGyro->fYp[Z] - pthisSV->fbPl[Z];

	// initialize the a priori orientation quaternion to the previous a posteriori estimate
	pthisSV->fqMi = pthisSV->fqPl;

	// integrate the buffered high frequency (typically 200Hz) gyro readings
	for (j = 0; j < iOverSampleRatio; j++)
	{
		// compute the incremental fast (typically 200Hz) rotation vector rvec (deg)
		for (i = X; i <= Z; i++)
		{
			rvec[i] = (((float)pthisGyro->iYpFast[j][i] * pthisGyro->fDegPerSecPerCount) - pthisSV->fbPl[i]) * pthisSV->fFastdeltat;
		}

		// compute the incremental quaternion fDeltaq from the rotation vector
		fQuaternionFromRotationVectorDeg(&(pthisSV->fDeltaq), rvec, 1.0F);

		// incrementally rotate the a priori orientation quaternion fqMi
		// the a posteriori quaternion fqPl is re-normalized later so this update is stable
		qAeqAxB(&(pthisSV->fqMi), &(pthisSV->fDeltaq));
	}

	// get the a priori rotation matrix from the a priori quaternion
	fRotationMatrixFromQuaternion(pthisSV->fRMi, &(pthisSV->fqMi));

	// *********************************************************************************
	// calculate a priori gyro, accelerometer and magnetometer estimates
	// of the gravity and geomagnetic vectors and errors
	// the most recent 'Fast' measurements are used to reduce phase errors
	// *********************************************************************************

	for (i = X; i <= Z; i++)
	{
		// compute the a priori gyro estimate of the gravitational vector (g, sensor frame)
		// using an absolute rotation of the global frame gravity vector (with magnitude 1g)
		if (ithisCoordSystem == NED)
		{
			// NED gravity is along positive z axis
			pthisSV->fgSeGyMi[i] = pthisSV->fRMi[i][Z];
		}
		else
		{
			// Android and Win8 gravity are along negative z axis
			pthisSV->fgSeGyMi[i] = -pthisSV->fRMi[i][Z];
		}

		// compute a priori acceleration (a-) (g, sensor frame) from decayed a posteriori estimate (g, sensor frame)
		pthisSV->faSeMi[i] = FCA_9DOF_GBY_KALMAN * pthisSV->faSePl[i];

		// compute the a priori gravity error vector (accelerometer minus gyro estimates) (g, sensor frame)
		if ((ithisCoordSystem == NED) || (ithisCoordSystem == WIN8))
		{
			// NED and Windows 8 have positive sign for gravity: y = g - a and g = y + a
			pthisSV->fgErrSeMi[i] = pthisAccel->fGpFast[i] + pthisSV->faSeMi[i] - pthisSV->fgSeGyMi[i];
		}
		else
		{
			// Android has negative sign for gravity: y = a - g, g = -y + a
			pthisSV->fgErrSeMi[i] = -pthisAccel->fGpFast[i] + pthisSV->faSeMi[i] - pthisSV->fgSeGyMi[i];
		}

		// compute the a priori gyro estimate of the geomagnetic vector (uT, sensor frame)
		// using an absolute rotation of the global frame geomagnetic vector (with magnitude B uT)
		if (ithisCoordSystem == NED)
		{
			// NED y component of geomagnetic vector in global frame is zero
			pthisSV->fmSeGyMi[i] = pthisSV->fRMi[i][X] * pthisSV->fmGl[X] + pthisSV->fRMi[i][Z] * pthisSV->fmGl[Z];
		}
		else
		{
			// Android and Windows 8 x component of geomagnetic vector in global frame is zero
			pthisSV->fmSeGyMi[i] = pthisSV->fRMi[i][Y] * pthisSV->fmGl[Y] + pthisSV->fRMi[i][Z] * pthisSV->fmGl[Z];
		}

		// compute the a priori geomagnetic error vector (magnetometer minus gyro estimates) (g, sensor frame)
		pthisSV->fmErrSeMi[i] = pthisMag->fBcFast[i] - pthisSV->fmSeGyMi[i];
	}

	// *********************************************************************************
	// update variable elements of measurement matrix C
	// *********************************************************************************

	// update measurement matrix C with -alpha(g-)x and -alpha(m-)x from gyro (g, uT, sensor frame)
	pthisSV->fC6x12[0][1] = FDEGTORAD * pthisSV->fgSeGyMi[Z];
	pthisSV->fC6x12[0][2] = -FDEGTORAD * pthisSV->fgSeGyMi[Y];
	pthisSV->fC6x12[1][2] = FDEGTORAD * pthisSV->fgSeGyMi[X];
	pthisSV->fC6x12[1][0] = -pthisSV->fC6x12[0][1];
	pthisSV->fC6x12[2][0] = -pthisSV->fC6x12[0][2];
	pthisSV->fC6x12[2][1] = -pthisSV->fC6x12[1][2];
	pthisSV->fC6x12[3][1] = FDEGTORAD * pthisSV->fmSeGyMi[Z];
	pthisSV->fC6x12[3][2] = -FDEGTORAD * pthisSV->fmSeGyMi[Y];
	pthisSV->fC6x12[4][2] = FDEGTORAD * pthisSV->fmSeGyMi[X];
	pthisSV->fC6x12[4][0] = -pthisSV->fC6x12[3][1];
	pthisSV->fC6x12[5][0] = -pthisSV->fC6x12[3][2];
	pthisSV->fC6x12[5][1] = -pthisSV->fC6x12[4][2];
	pthisSV->fC6x12[0][4] = -pthisSV->fdeltat * pthisSV->fC6x12[0][1];
	pthisSV->fC6x12[0][5] = -pthisSV->fdeltat * pthisSV->fC6x12[0][2];
	pthisSV->fC6x12[1][5] = -pthisSV->fdeltat * pthisSV->fC6x12[1][2];
	pthisSV->fC6x12[1][3]= -pthisSV->fC6x12[0][4];
	pthisSV->fC6x12[2][3]= -pthisSV->fC6x12[0][5];
	pthisSV->fC6x12[2][4]= -pthisSV->fC6x12[1][5];
	pthisSV->fC6x12[3][4] = -pthisSV->fdeltat * pthisSV->fC6x12[3][1];
	pthisSV->fC6x12[3][5] = -pthisSV->fdeltat * pthisSV->fC6x12[3][2];
	pthisSV->fC6x12[4][5] = -pthisSV->fdeltat * pthisSV->fC6x12[4][2];
	pthisSV->fC6x12[4][3] = -pthisSV->fC6x12[3][4];
	pthisSV->fC6x12[5][3] = -pthisSV->fC6x12[3][5];
	pthisSV->fC6x12[5][4] = -pthisSV->fC6x12[4][5];

	// *********************************************************************************
	// calculate the Kalman gain matrix K
	// K = P- * C^T * inv(C * P- * C^T + Qv) = Qw * C^T * inv(C * Qw * C^T + Qv)
	// Qw is used as a proxy for P- throughout the code
	// P+ is used here as a working array to reduce RAM usage and is re-computed later
	// *********************************************************************************

	// set ftmpA12x6 = P- * C^T = Qw * C^T where Qw and C are both sparse
	// C also has a significant number of +1 and -1 entries
	// ftmpA12x6 is also sparse but not symmetric
	for (i = 0; i < 12; i++) // loop over rows of ftmpA12x6
	{
		// initialize pftmpA12x6ij for current i, j=0
		pftmpA12x6ij = ftmpA12x6[i];

		for (j = 0; j < 6; j++) // loop over columns of ftmpA12x6
		{
			// zero ftmpA12x6[i][j]
			*pftmpA12x6ij = 0.0F;

			// initialize pfC6x12jk for current j, k=0
			pfC6x12jk = pthisSV->fC6x12[j];

			// initialize pfQw12x12ik for current i, k=0
			pfQw12x12ik = pthisSV->fQw12x12[i];

			// sum matrix products over inner loop over k
			for (k = 0; k < 12; k++)
			{
				if ((*pfQw12x12ik != 0.0F) && (*pfC6x12jk != 0.0F))
				{
					if (*pfC6x12jk == 1.0F)
						*pftmpA12x6ij += *pfQw12x12ik;
					else if (*pfC6x12jk == -1.0F)
						*pftmpA12x6ij -= *pfQw12x12ik;
					else
						*pftmpA12x6ij += *pfQw12x12ik * *pfC6x12jk;
				}

				// increment pfC6x12jk and pfQw12x12ik for next iteration of k
				pfC6x12jk++;
				pfQw12x12ik++;

			} // end of loop over k

			// increment pftmpA12x6ij for next iteration of j
			pftmpA12x6ij++;

		} // end of loop over j
	} // end of loop over i

	// set symmetric P+ (6x6 scratch sub-matrix) to C * P- * C^T + Qv
	// = C * (Qw * C^T) + Qv = C * ftmpA12x6 + Qv
	// both C and ftmpA12x6 are sparse but not symmetric
	for (i = 0; i < 6; i++) // loop over rows of P+
	{
		// initialize pfPPlus12x12ij for current i, j=i
		pfPPlus12x12ij = pthisSV->fPPlus12x12[i] + i;

		for (j = i; j < 6; j++) // loop over above diagonal columns of P+
		{
			// zero P+[i][j] 
			*pfPPlus12x12ij = 0.0F;

			// initialize pfC6x12ik for current i, k=0
			pfC6x12ik = pthisSV->fC6x12[i];

			// initialize pftmpA12x6kj for current j, k=0
			pftmpA12x6kj = *ftmpA12x6 + j;

			// sum matrix products over inner loop over k
			for (k = 0; k < 12; k++)
			{
				if ((*pfC6x12ik != 0.0F) && (*pftmpA12x6kj != 0.0F))
				{
					if (*pfC6x12ik == 1.0F)
						*pfPPlus12x12ij += *pftmpA12x6kj;
					else if (*pfC6x12ik == -1.0F)
						*pfPPlus12x12ij -= *pftmpA12x6kj;
					else
						*pfPPlus12x12ij += *pfC6x12ik * *pftmpA12x6kj;
				}

				// update pfC6x12ik and pftmpA12x6kj for next iteration of k
				pfC6x12ik++;						
				pftmpA12x6kj += 6;

			} // end of loop over k

			// increment pfPPlus12x12ij for next iteration of j
			pfPPlus12x12ij++;

		} // end of loop over j
	} // end of loop over i

	// add in noise covariance terms to the diagonal
	pthisSV->fPPlus12x12[0][0] +=  pthisSV->fQvAA;
	pthisSV->fPPlus12x12[1][1] +=  pthisSV->fQvAA;
	pthisSV->fPPlus12x12[2][2] +=  pthisSV->fQvAA;
	pthisSV->fPPlus12x12[3][3] +=  pthisSV->fQvMM;
	pthisSV->fPPlus12x12[4][4] +=  pthisSV->fQvMM;
	pthisSV->fPPlus12x12[5][5] +=  pthisSV->fQvMM;

	// copy above diagonal elements of P+ (6x6 scratch sub-matrix) to below diagonal
	for (i = 1; i < 6; i++) 	
		for (j = 0; j < i; j++) 
			pthisSV->fPPlus12x12[i][j] = pthisSV->fPPlus12x12[j][i];

	// calculate inverse of P+ (6x6 scratch sub-matrix) = inv(C * P- * C^T + Qv) = inv(C * Qw * C^T + Qv)
	for (i = 0; i < 6; i++)
	{
		pfRows[i] = pthisSV->fPPlus12x12[i];
	}
	fmatrixAeqInvA(pfRows, iColInd, iRowInd, iPivot, 3);

	// set K = P- * C^T * inv(C * P- * C^T + Qv) = Qw * C^T * inv(C * Qw * C^T + Qv)
	// = ftmpA12x6 * P+ (6x6 sub-matrix)
	// ftmpA12x6 = Qw * C^T is sparse but P+ (6x6 sub-matrix) is not
	// K is not symmetric because C is not symmetric
	for (i = 0; i < 12; i++) // loop over rows of K12x6
	{
		// initialize pfK12x6ij for current i, j=0
		pfK12x6ij = pthisSV->fK12x6[i];

		for (j = 0; j < 6; j++) // loop over columns of K12x6
		{
			// zero the matrix element fK12x6[i][j]
			*pfK12x6ij = 0.0F;

			// initialize pftmpA12x6ik for current i, k=0
			pftmpA12x6ik = ftmpA12x6[i];

			// initialize pfPPlus12x12kj for current j, k=0
			pfPPlus12x12kj = *pthisSV->fPPlus12x12 + j;

			// sum matrix products over inner loop over k
			for (k = 0; k < 6; k++)
			{
				if (*pftmpA12x6ik != 0.0F)
				{
					*pfK12x6ij += *pftmpA12x6ik * *pfPPlus12x12kj;
				}

				// increment pftmpA12x6ik and pfPPlus12x12kj for next iteration of k
				pftmpA12x6ik++;
				pfPPlus12x12kj += 12;

			} // end of loop over k

			// increment pfK12x6ij for the next iteration of j
			pfK12x6ij++;

		} // end of loop over j
	} // end of loop over i

	// *********************************************************************************
	// calculate a posteriori error estimate: xe+ = K * ze-
	// *********************************************************************************

	// first calculate all four error vector components using accelerometer error component only
	// for fThErrPl, fbErrPl, faErrSePl but also magnetometer for fdErrSePl
	for (i = X; i <= Z; i++)
	{
		pthisSV->fThErrPl[i] = pthisSV->fK12x6[i][0] * pthisSV->fgErrSeMi[X] +
				pthisSV->fK12x6[i][1] * pthisSV->fgErrSeMi[Y] +
				pthisSV->fK12x6[i][2] * pthisSV->fgErrSeMi[Z];
		pthisSV->fbErrPl[i] = pthisSV->fK12x6[i + 3][0] * pthisSV->fgErrSeMi[X] + 
				pthisSV->fK12x6[i + 3][1] * pthisSV->fgErrSeMi[Y] + 
				pthisSV->fK12x6[i + 3][2] * pthisSV->fgErrSeMi[Z];
		pthisSV->faErrSePl[i] = pthisSV->fK12x6[i + 6][0] * pthisSV->fgErrSeMi[X] +
				pthisSV->fK12x6[i + 6][1] * pthisSV->fgErrSeMi[Y] +
				pthisSV->fK12x6[i + 6][2] * pthisSV->fgErrSeMi[Z];
		pthisSV->fdErrSePl[i] = pthisSV->fK12x6[i + 9][0] * pthisSV->fgErrSeMi[X] +
				pthisSV->fK12x6[i + 9][1] * pthisSV->fgErrSeMi[Y] +
				pthisSV->fK12x6[i + 9][2] * pthisSV->fgErrSeMi[Z] +
				pthisSV->fK12x6[i + 9][3] * pthisSV->fmErrSeMi[X] +
				pthisSV->fK12x6[i + 9][4] * pthisSV->fmErrSeMi[Y] +
				pthisSV->fK12x6[i + 9][5] * pthisSV->fmErrSeMi[Z];
	}

	// set the magnetic jamming flag if there is a significant magnetic error power after calibration
	ftmp = pthisSV->fdErrSePl[X] * pthisSV->fdErrSePl[X] + pthisSV->fdErrSePl[Y] * pthisSV->fdErrSePl[Y] +
			pthisSV->fdErrSePl[Z] * pthisSV->fdErrSePl[Z];
	iMagJamming = (pthisMagCal->iValidMagCal) && (ftmp > pthisMagCal->fFourBsq);

	// add the remaining magnetic error terms if there is calibration and no magnetic jamming
	if (pthisMagCal->iValidMagCal && !iMagJamming)
	{
		for (i = X; i <= Z; i++)
		{
			pthisSV->fThErrPl[i] += pthisSV->fK12x6[i][3] * pthisSV->fmErrSeMi[X] +
					pthisSV->fK12x6[i][4] * pthisSV->fmErrSeMi[Y] +
					pthisSV->fK12x6[i][5] * pthisSV->fmErrSeMi[Z];
			pthisSV->fbErrPl[i] += pthisSV->fK12x6[i + 3][3] * pthisSV->fmErrSeMi[X] +
					pthisSV->fK12x6[i + 3][4] * pthisSV->fmErrSeMi[Y] +
					pthisSV->fK12x6[i + 3][5] * pthisSV->fmErrSeMi[Z]; 
			pthisSV->faErrSePl[i] += pthisSV->fK12x6[i + 6][3] * pthisSV->fmErrSeMi[X] +
					pthisSV->fK12x6[i + 6][4] * pthisSV->fmErrSeMi[Y] +
					pthisSV->fK12x6[i + 6][5] * pthisSV->fmErrSeMi[Z]; 
		}
	}

	// *********************************************************************************
	// apply the a posteriori error corrections to the a posteriori state vector
	// *********************************************************************************

	// get the a posteriori delta quaternion
	fQuaternionFromRotationVectorDeg(&(pthisSV->fDeltaq), pthisSV->fThErrPl, -1.0F);

	// compute the a posteriori orientation quaternion fqPl = fqMi * Deltaq(-thetae+)
	// the resulting quaternion may have negative scalar component q0
	qAeqBxC(&(pthisSV->fqPl), &(pthisSV->fqMi), &(pthisSV->fDeltaq));

	// normalize the a posteriori orientation quaternion to stop error propagation 
	// the renormalization function ensures that the scalar component q0 is non-negative
	fqAeqNormqA(&(pthisSV->fqPl));

	// compute the a posteriori rotation matrix from the a posteriori quaternion
	fRotationMatrixFromQuaternion(pthisSV->fRPl, &(pthisSV->fqPl));

	// compute the rotation vector from the a posteriori quaternion
	fRotationVectorDegFromQuaternion(&(pthisSV->fqPl), pthisSV->fRVecPl);

	// update the a posteriori gyro offset vector b+ and
	// assign the entire linear acceleration error vector to update the linear acceleration
	for (i = X; i <= Z; i++)
	{
		// b+[k] = b-[k] - be+[k] = b+[k] - be+[k] (deg/s)
		pthisSV->fbPl[i] -= pthisSV->fbErrPl[i];
		// a+ = a- - ae+ (g, sensor frame)
		pthisSV->faSePl[i] = pthisSV->faSeMi[i] - pthisSV->faErrSePl[i];
	}

	// compute the linear acceleration in the global frame from the accelerometer measurement (sensor frame).
	// de-rotate the accelerometer measurement from the sensor to global frame using the inverse (transpose) 
	// of the a posteriori rotation matrix
	pthisSV->faGlPl[X] = pthisSV->fRPl[X][X] * pthisAccel->fGpFast[X] + pthisSV->fRPl[Y][X] * pthisAccel->fGpFast[Y] +
			pthisSV->fRPl[Z][X] * pthisAccel->fGpFast[Z];
	pthisSV->faGlPl[Y] = pthisSV->fRPl[X][Y] * pthisAccel->fGpFast[X] + pthisSV->fRPl[Y][Y] * pthisAccel->fGpFast[Y] +
			pthisSV->fRPl[Z][Y] * pthisAccel->fGpFast[Z];
	pthisSV->faGlPl[Z] = pthisSV->fRPl[X][Z] * pthisAccel->fGpFast[X] + pthisSV->fRPl[Y][Z] * pthisAccel->fGpFast[Y] +
			pthisSV->fRPl[Z][Z] * pthisAccel->fGpFast[Z];
	// remove gravity and correct the sign if the coordinate system is gravity positive / acceleration negative 
	switch (ithisCoordSystem)
	{
	case NED:
		// gravity positive NED
		pthisSV->faGlPl[X] = -pthisSV->faGlPl[X];
		pthisSV->faGlPl[Y] = -pthisSV->faGlPl[Y];
		pthisSV->faGlPl[Z] = -(pthisSV->faGlPl[Z] - 1.0F);
		break;
	case ANDROID:
		// acceleration positive ENU
		pthisSV->faGlPl[X] = pthisSV->faGlPl[X];
		pthisSV->faGlPl[Y] = pthisSV->faGlPl[Y];
		pthisSV->faGlPl[Z] = pthisSV->faGlPl[Z] - 1.0F;
		break;
	case WIN8:
	default:
		// gravity positive ENU
		pthisSV->faGlPl[X] = -pthisSV->faGlPl[X];
		pthisSV->faGlPl[Y] = -pthisSV->faGlPl[Y];
		pthisSV->faGlPl[Z] = -(pthisSV->faGlPl[Z] + 1.0F);
		break;
	}

	// update the reference geomagnetic vector using magnetic disturbance error if valid calibration and no jamming
	if (pthisMagCal->iValidMagCal && !iMagJamming)
	{
		if (ithisCoordSystem == NED)
		{
			// de-rotate the NED magnetic disturbance error de+ from the sensor to the global reference frame
			// using the inverse (transpose) of the a posteriori rotation matrix
			pthisSV->fdErrGlPl[X] = pthisSV->fRPl[X][X] * pthisSV->fdErrSePl[X] + pthisSV->fRPl[Y][X] * pthisSV->fdErrSePl[Y] +
					pthisSV->fRPl[Z][X] * pthisSV->fdErrSePl[Z];
			pthisSV->fdErrGlPl[Z] = pthisSV->fRPl[X][Z] * pthisSV->fdErrSePl[X] + pthisSV->fRPl[Y][Z] * pthisSV->fdErrSePl[Y] +
					pthisSV->fRPl[Z][Z] * pthisSV->fdErrSePl[Z];

			// compute components of the new geomagnetic vector
			// the north pointing component fadj must always be non-negative
			fopp = pthisSV->fmGl[Z] - pthisSV->fdErrGlPl[Z];
			fadj = pthisSV->fmGl[X] - pthisSV->fdErrGlPl[X];
			if (fadj < 0.0F)
			{
				fadj = 0.0F;
			}
			fhyp = sqrtf(fopp * fopp + fadj * fadj);

			// check for the pathological condition of zero geomagnetic field
			if (fhyp != 0.0F)
			{
				// compute the sine and cosine of the new geomagnetic vector
				ftmp = 1.0F / fhyp;
				fsindelta = fopp * ftmp;
				fcosdelta = fadj * ftmp;

				// limit the inclination angle between limits to prevent runaway
				if (fsindelta > SINDELTAMAX)
				{
					fsindelta = SINDELTAMAX;
					fcosdelta = COSDELTAMAX;
				}
				else if (fsindelta < -SINDELTAMAX)
				{
					fsindelta = -SINDELTAMAX;
					fcosdelta = COSDELTAMAX;
				}

				// compute the new geomagnetic vector (always north pointing)
				pthisSV->fDeltaPl = fasin_deg(fsindelta);
				pthisSV->fmGl[X] = pthisMagCal->fB * fcosdelta;
				pthisSV->fmGl[Z] = pthisMagCal->fB * fsindelta;
			} // end hyp == 0.0F
		} // end NED
		else
		{
			// de-rotate the Android and Windows 8 magnetic disturbance error de+ from the sensor to the global reference frame
			// using the inverse (transpose) of the a posteriori rotation matrix
			pthisSV->fdErrGlPl[Y] = pthisSV->fRPl[X][Y] * pthisSV->fdErrSePl[X] + pthisSV->fRPl[Y][Y] * pthisSV->fdErrSePl[Y] +
					pthisSV->fRPl[Z][Y] * pthisSV->fdErrSePl[Z];
			pthisSV->fdErrGlPl[Z] = pthisSV->fRPl[X][Z] * pthisSV->fdErrSePl[X] + pthisSV->fRPl[Y][Z] * pthisSV->fdErrSePl[Y] +
					pthisSV->fRPl[Z][Z] * pthisSV->fdErrSePl[Z];

			// compute components of the new geomagnetic vector
			// the north pointing component fadj must always be non-negative
			fopp = -(pthisSV->fmGl[Z] - pthisSV->fdErrGlPl[Z]);
			fadj = pthisSV->fmGl[Y] - pthisSV->fdErrGlPl[Y];
			if (fadj < 0.0F)
			{
				fadj = 0.0F;
			}
			fhyp = sqrtf(fopp * fopp + fadj * fadj);

			// check for the pathological condition of zero geomagnetic field
			if (fhyp != 0.0F)
			{
				// compute the sine and cosine of the new geomagnetic vector
				ftmp = 1.0F / fhyp;
				fsindelta = fopp * ftmp;
				fcosdelta = fadj * ftmp;

				// limit the inclination angle between limits to prevent runaway
				if (fsindelta > SINDELTAMAX)
				{
					fsindelta = SINDELTAMAX;
					fcosdelta = COSDELTAMAX;
				}
				else if (fsindelta < -SINDELTAMAX)
				{
					fsindelta = -SINDELTAMAX;
					fcosdelta = COSDELTAMAX;
				}

				// compute the new geomagnetic vector (always north pointing)
				pthisSV->fDeltaPl = fasin_deg(fsindelta);
				pthisSV->fmGl[Y] = pthisMagCal->fB * fcosdelta;
				pthisSV->fmGl[Z] = -pthisMagCal->fB * fsindelta;
			} // end hyp == 0.0F
		} // end Android and Win8
	} // end iValidMagCal

	// *********************************************************************************
	// compute the a posteriori Euler angles from the orientation matrix
	// *********************************************************************************

	if (ithisCoordSystem == NED)
	{
		// calculate the NED Euler angles
		fNEDAnglesDegFromRotationMatrix(pthisSV->fRPl, &(pthisSV->fPhiPl), &(pthisSV->fThePl), &(pthisSV->fPsiPl),
				&(pthisSV->fRhoPl), &(pthisSV->fChiPl));
	}
	else if (ithisCoordSystem == ANDROID)
	{
		// calculate the Android Euler angles
		fAndroidAnglesDegFromRotationMatrix(pthisSV->fRPl, &(pthisSV->fPhiPl), &(pthisSV->fThePl), &(pthisSV->fPsiPl),
				&(pthisSV->fRhoPl), &(pthisSV->fChiPl));
	}
	else
	{
		// calculate Win8 Euler angles
		fWin8AnglesDegFromRotationMatrix(pthisSV->fRPl, &(pthisSV->fPhiPl), &(pthisSV->fThePl), &(pthisSV->fPsiPl),
				&(pthisSV->fRhoPl), &(pthisSV->fChiPl));
	}

	// ***********************************************************************************
	// calculate (symmetric) a posteriori error covariance matrix P+
	// P+ = (I12 - K * C) * P- = (I12 - K * C) * Qw = Qw - K * (C * Qw)
	// both Qw and P+ are used as working arrays in this section
	// at the end of this section, P+ is valid but Qw is over-written
	// ***********************************************************************************

	// set P+ (6x12 scratch sub-matrix) to the product C (6x12) * Qw (12x12)
	// where both C and Qw are sparse and C has a significant number of +1 and -1 entries
	// the resulting matrix is sparse but not symmetric
	for (i = 0; i < 6; i++)
	{
		// initialize pfPPlus12x12ij for current i, j=0
		pfPPlus12x12ij = pthisSV->fPPlus12x12[i];

		for (j = 0; j < 12; j++)
		{
			// zero P+[i][j] 
			*pfPPlus12x12ij = 0.0F;

			// initialize pfC6x12ik for current i, k=0
			pfC6x12ik = pthisSV->fC6x12[i];

			// initialize pfQw12x12kj for current j, k=0
			pfQw12x12kj = &pthisSV->fQw12x12[0][j];

			// sum matrix products over inner loop over k
			for (k = 0; k < 12; k++)
			{
				if ((*pfC6x12ik != 0.0F) && (*pfQw12x12kj != 0.0F))
				{
					if (*pfC6x12ik == 1.0F)
						*pfPPlus12x12ij += *pfQw12x12kj;
					else if (*pfC6x12ik == -1.0F)
						*pfPPlus12x12ij -= *pfQw12x12kj;
					else
						*pfPPlus12x12ij += *pfC6x12ik * *pfQw12x12kj;
				}

				// update pfC6x12ik and pfQw12x12kj for next iteration of k
				pfC6x12ik++;
				pfQw12x12kj += 12;

			} // end of loop over k

			// increment pfPPlus12x12ij for next iteration of j
			pfPPlus12x12ij++;

		} // end of loop over j
	} // end of loop over i

	// compute P+ = (I12 - K * C) * Qw = Qw - K * (C * Qw) = Qw - K * P+ (6x12 sub-matrix)
	// storing result P+ in Qw and over-writing Qw which is OK since Qw is later computed from P+
	// where working array P+ (6x12 sub-matrix) is sparse but K is not sparse
	// only on and above diagonal terms of P+ are computed since P+ is symmetric
	for (i = 0; i < 12; i++)
	{
		// initialize pfQw12x12ij for current i, j=i
		pfQw12x12ij = pthisSV->fQw12x12[i] + i;

		for (j = i; j < 12; j++)
		{
			// initialize pfK12x6ik for current i, k=0
			pfK12x6ik = pthisSV->fK12x6[i];

			// initialize pfPPlus12x12kj for current j, k=0
			pfPPlus12x12kj = *pthisSV->fPPlus12x12 + j;

			// compute on and above diagonal matrix entry
			for (k = 0; k < 6; k++)
			{
				// check for non-zero values since P+ (6x12 scratch sub-matrix) is sparse
				if (*pfPPlus12x12kj != 0.0F)
				{
					*pfQw12x12ij -= *pfK12x6ik * *pfPPlus12x12kj;
				}

				// increment pfK12x6ik and pfPPlus12x12kj for next iteration of k
				pfK12x6ik++;
				pfPPlus12x12kj += 12;

			} // end of loop over k

			// increment pfQw12x12ij for next iteration of j
			pfQw12x12ij++;

		} // end of loop over j
	} // end of loop over i

	// Qw now holds the on and above diagonal elements of P+
	// so perform a simple copy to the all elements of P+
	// after execution of this code P+ is valid but Qw remains invalid
	for (i = 0; i < 12; i++)
	{
		// initialize pfPPlus12x12ij and pfQw12x12ij for current i, j=i
		pfPPlus12x12ij = pthisSV->fPPlus12x12[i] + i;
		pfQw12x12ij = pthisSV->fQw12x12[i] + i;

		// copy the on-diagonal elements and increment pointers to enter loop at j=i+1
		*(pfPPlus12x12ij++) = *(pfQw12x12ij++);

		// loop over above diagonal columns j copying to below-diagonal elements
		for (j = i + 1; j < 12; j++)
		{
			*(pfPPlus12x12ij++)= pthisSV->fPPlus12x12[j][i] = *(pfQw12x12ij++);
		}
	}

	// *********************************************************************************
	// re-create the noise covariance matrix Qw=fn(P+) for the next iteration
	// using the elements of P+ which are now valid
	// Qw was over-written earlier but is here recomputed (all elements)
	// *********************************************************************************

	// zero the matrix Qw
	for (i = 0; i < 12; i++)
	{
		for (j = 0; j < 12; j++)
		{
			pthisSV->fQw12x12[i][j] = 0.0F;
		}
	}

	// update the covariance matrix components
	for (i = 0; i < 3; i++)
	{
		// Qw[th-th-] = Qw[0-2][0-2] = E[th-(th-)^T] = Q[th+th+] + deltat^2 * (Q[b+b+] + (Qwb + QvG) * I)
		pthisSV->fQw12x12[i][i] = pthisSV->fPPlus12x12[i][i] + pthisSV->fdeltatsq * (pthisSV->fPPlus12x12[i + 3][i + 3] + pthisSV->fQwbplusQvG);

		// Qw[b-b-] = Qw[3-5][3-5] = E[b-(b-)^T] = Q[b+b+] + Qwb * I
		pthisSV->fQw12x12[i + 3][i + 3] = pthisSV->fPPlus12x12[i + 3][i + 3] + FQWB_9DOF_GBY_KALMAN;

		// Qw[th-b-] = Qw[0-2][3-5] = E[th-(b-)^T] = -deltat * (Q[b+b+] + Qwb * I) = -deltat * Qw[b-b-]
		pthisSV->fQw12x12[i][i + 3] = pthisSV->fQw12x12[i + 3][i] = -pthisSV->fdeltat * pthisSV->fQw12x12[i + 3][i + 3];

		// Qw[a-a-] = Qw[6-8][6-8] = E[a-(a-)^T] = ca^2 * Q[a+a+] + Qwa * I
		pthisSV->fQw12x12[i + 6][i + 6] = pthisSV->fcasq * pthisSV->fPPlus12x12[i + 6][i + 6] + FQWA_9DOF_GBY_KALMAN;

		// Qw[d-d-] = Qw[9-11][9-11] = E[d-(d-)^T] = cd^2 * Q[d+d+] + Qwd * I
		pthisSV->fQw12x12[i + 9][i + 9] = pthisSV->fcdsq * pthisSV->fPPlus12x12[i + 9][i + 9] + FQWD_9DOF_GBY_KALMAN;
	}

	return;
}  // end fRun_9DOF_GBY_KALMAN

