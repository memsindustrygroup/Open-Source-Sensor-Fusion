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
#ifndef FUSION_H
#define FUSION_H

// *********************************************************************************
// COMPUTE_6DOF_GY_KALMAN constants
// *********************************************************************************
// kalman filter noise variances
#define FQVA_6DOF_GY_KALMAN 2E-6F				// accelerometer noise g^2 so 1.4mg RMS
#define FQVG_6DOF_GY_KALMAN 0.3F				// gyro noise (deg/s)^2
#define FQWB_6DOF_GY_KALMAN 1E-9F				// gyro offset drift (deg/s)^2: 1E-9 implies 0.09deg/s max at 50Hz
#define FQWA_6DOF_GY_KALMAN 1E-4F				// linear acceleration drift g^2 (increase slows convergence to g but reduces sensitivity to shake)
// initialization of Qw covariance matrix
#define FQWINITTHTH_6DOF_GY_KALMAN 2000E-5F		// th_e * th_e terms
#define FQWINITBB_6DOF_GY_KALMAN 250E-3F		// for FXAS21000: b_e * b_e terms
#define FQWINITTHB_6DOF_GY_KALMAN 0.0F			// th_e * b_e terms
#define FQWINITAA_6DOF_GY_KALMAN 10E-5F			// a_e * a_e terms (increase slows convergence to g but reduces sensitivity to shake)
// linear acceleration time constant
#define FCA_6DOF_GY_KALMAN 0.5F					// linear acceleration decay factor

// *********************************************************************************
// COMPUTE_9DOF_GBY_KALMAN constants
// *********************************************************************************
// kalman filter noise variances
#define FQVA_9DOF_GBY_KALMAN 2E-6F				// accelerometer noise g^2 so 1.4mg RMS
#define FQVM_9DOF_GBY_KALMAN 0.1F				// magnetometer noise uT^2
#define FQVG_9DOF_GBY_KALMAN 0.3F				// gyro noise (deg/s)^2
#define FQWB_9DOF_GBY_KALMAN 1E-9F				// gyro offset drift (deg/s)^2: 1E-9 implies 0.09deg/s max at 50Hz
#define FQWA_9DOF_GBY_KALMAN 1E-4F				// linear acceleration drift g^2 (increase slows convergence to g but reduces sensitivity to shake)
#define FQWD_9DOF_GBY_KALMAN 0.5F				// magnetic disturbance drift uT^2 (increase slows convergence to B but reduces sensitivity to magnet)
// initialization of Qw covariance matrix
#define FQWINITTHTH_9DOF_GBY_KALMAN 2000E-5F	// th_e * th_e terms
#define FQWINITBB_9DOF_GBY_KALMAN 250E-3F		// b_e * b_e terms
#define FQWINITTHB_9DOF_GBY_KALMAN 0.0F			// th_e * b_e terms
#define FQWINITAA_9DOF_GBY_KALMAN 10E-5F		// a_e * a_e terms (increase slows convergence to g but reduces sensitivity to shake)
#define FQWINITDD_9DOF_GBY_KALMAN 600E-3F		// d_e * d_e terms (increase slows convergence to B but reduces sensitivity to magnet)
// linear acceleration and magnetic disturbance time constants
#define FCA_9DOF_GBY_KALMAN 0.5F				// linear acceleration decay factor
#define FCD_9DOF_GBY_KALMAN 0.5F				// magnetic disturbance decay factor
// maximum geomagnetic inclination angle tracked by Kalman filter
#define SINDELTAMAX 0.9063078F		// sin of max +ve geomagnetic inclination angle: here 65.0 deg
#define COSDELTAMAX 0.4226183F		// cos of max +ve geomagnetic inclination angle: here 65.0 deg

// *********************************************************************************
// function prototypes
// *********************************************************************************
void fInit_1DOF_P_BASIC(struct SV_1DOF_P_BASIC *pthisSV, float flpftimesecs, int16 iSensorFS, int16 iOverSampleRatio);
void fInit_3DOF_G_BASIC(struct SV_3DOF_G_BASIC *pthisSV, float flpftimesecs, int16 iSensorFS, int16 iOverSampleRatio);
void fInit_3DOF_B_BASIC(struct SV_3DOF_B_BASIC *pthisSV, float flpftimesecs, int16 iSensorFS, int16 iOverSampleRatio);
void fInit_3DOF_Y_BASIC(struct SV_3DOF_Y_BASIC *pthisSV, int16 iSensorFS, int16 iOverSampleRatio);
void fInit_6DOF_GB_BASIC(struct SV_6DOF_GB_BASIC *pthisSV, float flpftimesecs, int16 iSensorFS, int16 iOverSampleRatio);
void fInit_6DOF_GY_KALMAN(struct SV_6DOF_GY_KALMAN *pthisSV, int16 iSensorFS, int16 iOverSampleRatio);
void fInit_9DOF_GBY_KALMAN(struct SV_9DOF_GBY_KALMAN *pthisSV, int16 ithisCoordSystem, int16 iSensorFS, int16 iOverSampleRatio);

void fRun_1DOF_P_BASIC(struct SV_1DOF_P_BASIC *pthisSV, struct PressureSensor *pthisPressure, int32 loopcounter);
void fRun_3DOF_G_BASIC(struct SV_3DOF_G_BASIC *pthisSV, struct AccelSensor *pthisAccel, int32 loopcounter, int16 ithisCoordSystem);
void fRun_3DOF_B_BASIC(struct SV_3DOF_B_BASIC *pthisSV, struct MagSensor *pthisMag, int32 loopcounter, int16 ithisCoordSystem);
void fRun_3DOF_Y_BASIC(struct SV_3DOF_Y_BASIC *pthisSV, struct GyroSensor *pthisGyro, int32 loopcounter, int16 ithisCoordSystem, int16 iOverSampleRatio);
void fRun_6DOF_GB_BASIC(struct SV_6DOF_GB_BASIC *pthisSV, struct MagSensor *pthisMag, struct AccelSensor *pthisAccel, int32 loopcounter, int16 ithisCoordSystem);
void fRun_6DOF_GY_KALMAN(struct SV_6DOF_GY_KALMAN *pthisSV, struct AccelSensor *pthisAccel, struct GyroSensor *pthisGyro, int16 ithisCoordSystem, int16 iOverSampleRatio);
void fRun_9DOF_GBY_KALMAN(struct SV_9DOF_GBY_KALMAN *pthisSV, struct AccelSensor *pthisAccel, struct MagSensor *pthisMag, struct GyroSensor *pthisGyro,
		struct MagCalibration *pthisMagCal, int16 ithisCoordSystem, int16 iOverSampleRatio);

#endif   // #ifndef FUSION_H
