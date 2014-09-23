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
#ifndef TASKS_H
#define TASKS_H

// structure definitions

// 1DOF pressure state vector structure
struct SV_1DOF_P_BASIC
{
	float fLPHp;					// low pass filtered height (m)
	float fLPTp;					// low pass filtered temperature (C);				
	float fdeltat;					// time interval (s)
	float flpf;						// low pass filter coefficient
	int32 systick;					// systick timer
	int8 resetflag;					// flag to request re-initialization on next pass
};

// 3DOF basic accelerometer state vector structure
struct SV_3DOF_G_BASIC
{
	// start: elements common to all motion state vectors
	// Euler angles
	float fLPPhi;					// low pass roll (deg)
	float fLPThe;					// low pass pitch (deg)
	float fLPPsi;					// low pass yaw (deg)
	float fLPRho;					// low pass compass (deg)
	float fLPChi;					// low pass tilt from vertical (deg)
	// orientation matrix, quaternion and rotation vector
	float fLPR[3][3];				// low pass filtered orientation matrix
	struct fquaternion fLPq;		// low pass filtered orientation quaternion
	float fLPRVec[3];				// rotation vector
	// angular velocity
	float fOmega[3];				// angular velocity (deg/s)
	// systick timer for benchmarking
	int32 systick;					// systick timer
	// end: elements common to all motion state vectors
	float fR[3][3];					// unfiltered orientation matrix
	struct fquaternion fq;			// unfiltered orientation quaternion
	float fdeltat;					// time interval (s)
	float flpf;						// low pass filter coefficient
	int8 resetflag;					// flag to request re-initialization on next pass
};

// 3DOF basic magnetometer state vector structure
struct SV_3DOF_B_BASIC
{
	// start: elements common to all motion state vectors
	// Euler angles
	float fLPPhi;						// low pass roll (deg)
	float fLPThe;						// low pass pitch (deg)
	float fLPPsi;						// low pass yaw (deg)
	float fLPRho;						// low pass compass (deg)
	float fLPChi;						// low pass tilt from vertical (deg)
	// orientation matrix, quaternion and rotation vector
	float fLPR[3][3];					// low pass filtered orientation matrix
	struct fquaternion fLPq;			// low pass filtered orientation quaternion
	float fLPRVec[3];					// rotation vector
	// angular velocity
	float fOmega[3];					// angular velocity (deg/s)
	// systick timer for benchmarking
	int32 systick;						// systick timer
	// end: elements common to all motion state vectors
	float fR[3][3];						// unfiltered orientation matrix
	struct fquaternion fq;				// unfiltered orientation quaternion
	float fdeltat;						// time interval (s)
	float flpf;							// low pass filter coefficient
	int8 resetflag;					// flag to request re-initialization on next pass
};

// 3DOF basic gyroscope state vector structure
struct SV_3DOF_Y_BASIC
{
	// start: elements common to all motion state vectors
	// Euler angles
	float fPhi;						// roll (deg)
	float fThe;						// pitch (deg)
	float fPsi;						// yaw (deg)
	float fRho;						// compass (deg)
	float fChi;						// tilt from vertical (deg)
	// orientation matrices and quaternions
	float fR[3][3];					// unfiltered orientation matrix
	struct fquaternion fq;			// unfiltered orientation quaternion
	float fRVec[3];					// rotation vector
	// angular velocity
	float fOmega[3];				// angular velocity (deg/s)
	// systick timer for benchmarking
	int32 systick;					// systick timer
	// end: elements common to all motion state vectors
	struct fquaternion fDeltaq;		// delta rotation quaternion
	float fFastdeltat;				// sensor sampling interval (s) = 1 / SENSORFS
	float fdeltat;					// kalman filter sampling interval (s) = OVERSAMPLE_RATIO / SENSORFS
	int8 resetflag;					// flag to request re-initialization on next pass
};

// 6DOF basic accelerometer and magnetometer state vector structure
struct SV_6DOF_GB_BASIC
{
	// start: elements common to all motion state vectors
	// Euler angles
	float fLPPhi;					// low pass roll (deg)
	float fLPThe;					// low pass pitch (deg)
	float fLPPsi;					// low pass yaw (deg)
	float fLPRho;					// low pass compass (deg)
	float fLPChi;					// low pass tilt from vertical (deg)
	// orientation matrix, quaternion and rotation vector
	float fLPR[3][3];				// low pass filtered orientation matrix
	struct fquaternion fLPq;		// low pass filtered orientation quaternion
	float fLPRVec[3];				// rotation vector
	// angular velocity
	float fOmega[3];				// virtual gyro angular velocity (deg/s)
	// systick timer for benchmarking
	int32 systick;					// systick timer
	// end: elements common to all motion state vectors
	float fR[3][3];					// unfiltered orientation matrix
	struct fquaternion fq;			// unfiltered orientation quaternion
	float fDelta;					// unfiltered inclination angle (deg)
	float fLPDelta;					// low pass filtered inclination angle (deg)
	float fdeltat;					// time interval (s)
	float flpf;						// low pass filter coefficient
	int8 resetflag;					// flag to request re-initialization on next pass
};

// 6DOF Kalman filter accelerometer and gyroscope state vector structure
struct SV_6DOF_GY_KALMAN
{
	// start: elements common to all motion state vectors
	// Euler angles
	float fPhiPl;					// roll (deg)
	float fThePl;					// pitch (deg)
	float fPsiPl;					// yaw (deg)
	float fRhoPl;					// compass (deg)
	float fChiPl;					// tilt from vertical (deg)
	// orientation matrix, quaternion and rotation vector
	float fRPl[3][3];				// a posteriori  rotation matrix
	struct fquaternion fqPl;		// a posteriori orientation quaternion
	float fRVecPl[3];				// rotation vector
	// angular velocity
	float fOmega[3];				// angular velocity (deg/s)
	// systick timer for benchmarking
	int32 systick;					// systick timer
	// end: elements common to all motion state vectors
	
	// elements transmitted over bluetooth in kalman packet
	float fbPl[3];					// gyro offset (deg/s)
	float fThErrPl[3];				// orientation error (deg)
	float fbErrPl[3];				// gyro offset error (deg/s)
	// end elements transmitted in kalman packet
	
	float fzErrMi[3];				// angular error (deg) between a priori and eCompass orientations
	float fRMi[3][3];				// a priori rotation matrix
	struct fquaternion fqMi;		// a priori orientation quaternion
	struct fquaternion fDeltaq;		// delta a priori or a posteriori quaternion
	float faSePl[3];				// linear acceleration (g, sensor frame)
	float faErrSePl[3];				// linear acceleration error (g, sensor frame)
	float fgErrSeMi[3];				// difference (g, sensor frame) of gravity vector (accel) and gravity vector (gyro)
	float fgSeGyMi[3];				// gravity vector (g, sensor frame) measurement from gyro
	float faSeMi[3];				// linear acceleration (g, sensor frame)
	float fQvAA;					// accelerometer terms of Qv
	float fPPlus9x9[9][9];			// covariance matrix P+
	float fK9x3[9][3];				// kalman filter gain matrix K
	float fQw9x9[9][9];				// covariance matrix Qw
	float fC3x9[3][9];				// measurement matrix C
	float fcasq;					// FCA * FCA;
	float fFastdeltat;				// sensor sampling interval (s) = 1 / SENSORFS
	float fdeltat;					// kalman filter sampling interval (s) = OVERSAMPLE_RATIO / SENSORFS
	float fdeltatsq;				// fdeltat * fdeltat;
	float fQwbplusQvG;				// FQWB + FQVG;
	int16 iFirstOrientationLock;	// denotes that 6DOF orientation has locked to 3DOF
	int8 resetflag;					// flag to request re-initialization on next pass
};

// 9DOF Kalman filter accelerometer, magnetometer and gyroscope state vector structure
struct SV_9DOF_GBY_KALMAN
{
	// start: elements common to all motion state vectors
	// Euler angles
	float fPhiPl;					// roll (deg)
	float fThePl;					// pitch (deg)
	float fPsiPl;					// yaw (deg)
	float fRhoPl;					// compass (deg)
	float fChiPl;					// tilt from vertical (deg)
	// orientation matrix, quaternion and rotation vector
	float fRPl[3][3];				// a posteriori orientation matrix
	struct fquaternion fqPl;		// a posteriori orientation quaternion
	float fRVecPl[3];				// rotation vector
	// angular velocity
	float fOmega[3];				// angular velocity (deg/s)
	// systick timer for benchmarking
	int32 systick;					// systick timer;
	// end: elements common to all motion state vectors
	
	// elements transmitted over bluetooth in kalman packet
	float fbPl[3];					// gyro offset (deg/s)
	float fThErrPl[3];				// orientation error (deg)
	float fbErrPl[3];				// gyro offset error (deg/s)
	// end elements transmitted in kalman packet
	
	float fdErrGlPl[3];				// magnetic disturbance error (uT, global frame)
	float fdErrSePl[3];				// magnetic disturbance error (uT, sensor frame)
	float faErrSePl[3];				// linear acceleration error (g, sensor frame)
	float faSeMi[3];				// linear acceleration (g, sensor frame)
	float fDeltaPl;					// inclination angle (deg)
	float faSePl[3];				// linear acceleration (g, sensor frame)
	float faGlPl[3];				// linear acceleration (g, global frame)
	float fgErrSeMi[3];				// difference (g, sensor frame) of gravity vector (accel) and gravity vector (gyro)
	float fmErrSeMi[3];				// difference (uT, sensor frame) of geomagnetic vector (magnetometer) and geomagnetic vector (gyro)
	float fgSeGyMi[3];				// gravity vector (g, sensor frame) measurement from gyro
	float fmSeGyMi[3];				// geomagnetic vector (uT, sensor frame) measurement from gyro
	float fmGl[3];					// geomagnetic vector (uT, global frame)
	float fQvAA;					// accelerometer terms of Qv
	float fQvMM;					// magnetometer terms of Qv
	float fPPlus12x12[12][12];		// covariance matrix P+
	float fK12x6[12][6];			// kalman filter gain matrix K
	float fQw12x12[12][12];			// covariance matrix Qw
	float fC6x12[6][12];			// measurement matrix C
	float fRMi[3][3];				// a priori orientation matrix
	struct fquaternion fDeltaq;		// delta quaternion
	struct fquaternion fqMi;		// a priori orientation quaternion
	float fcasq;					// FCA * FCA;
	float fcdsq;					// FCD * FCD;
	float fFastdeltat;				// sensor sampling interval (s) = 1 / SENSORFS
	float fdeltat;					// kalman filter sampling interval (s) = OVERSAMPLE_RATIO / SENSORFS
	float fdeltatsq;				// fdeltat * fdeltat
	float fQwbplusQvG;				// FQWB + FQVG
	int16 iFirstOrientationLock;	// denotes that 9DOF orientation has locked to 6DOF
	int8 resetflag;					// flag to request re-initialization on next pass
};

// globals defined in tasks_func.c declared here for use elsewhere
extern struct PressureSensor thisPressure;   
extern struct AccelSensor thisAccel;   
extern struct MagSensor thisMag;       
extern struct GyroSensor thisGyro;     
extern struct MagCalibration thisMagCal;
extern struct MagneticBuffer thisMagBuffer;
extern struct SV_1DOF_P_BASIC thisSV_1DOF_P_BASIC;
extern struct SV_3DOF_G_BASIC thisSV_3DOF_G_BASIC;
extern struct SV_3DOF_B_BASIC thisSV_3DOF_B_BASIC;
extern struct SV_3DOF_Y_BASIC thisSV_3DOF_Y_BASIC;
extern struct SV_6DOF_GB_BASIC thisSV_6DOF_GB_BASIC;
extern struct SV_6DOF_GY_KALMAN thisSV_6DOF_GY_KALMAN;
extern struct SV_9DOF_GBY_KALMAN thisSV_9DOF_GBY_KALMAN;

// function prototypes for functions in tasks_func.c
void ApplyAccelHAL(struct AccelSensor *pthisAccel);
void ApplyMagHAL(struct MagSensor *pthisMag);
void ApplyGyroHAL(struct GyroSensor *pthisGyro, int16 irow);
void RdSensData_Init(void);
void RdSensData_Run(void);
void Fusion_Init(void);
void Fusion_Run(void);
void MagCal_Run(struct MagCalibration *pthisMagCal, struct MagneticBuffer *pthisMagBuffer);

#endif   // #ifndef TASKS_H
