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
// This file contains functions designed to operate on, or compute, orientations.
// These may be in rotation matrix form, quaternion form, or Euler angles.
// It also includes functions designed to operate with specify reference frames
// (Android, Windows 8, NED).
//
#include "Events.h"
#include "include_all.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "time.h"
#include "string.h"

// compile time constants that are private to this file
#define SMALLQ0 0.01F		// limit of quaternion scalar component requiring special algorithm
#define CORRUPTQUAT 0.001F	// threshold for deciding rotation quaternion is corrupt
#define SMALLMODULUS 0.01F	// limit where rounding errors may appear

// Aerospace NED accelerometer 3DOF tilt function computing rotation matrix fR
void f3DOFTiltNED(float fR[][3], float fGp[])
{
	// the NED self-consistency twist occurs at 90 deg pitch
	
	// local variables
	int16 i;				// counter
	float fmodGxyz;			// modulus of the x, y, z accelerometer readings
	float fmodGyz;			// modulus of the y, z accelerometer readings
	float frecipmodGxyz;	// reciprocal of modulus
	float ftmp;				// scratch variable

	// compute the accelerometer squared magnitudes
	fmodGyz = fGp[Y] * fGp[Y] + fGp[Z] * fGp[Z];
	fmodGxyz = fmodGyz + fGp[X] * fGp[X];

	// check for freefall special case where no solution is possible
	if (fmodGxyz == 0.0F)
	{
		f3x3matrixAeqI(fR);
		return;
	}

	// check for vertical up or down gimbal lock case
	if (fmodGyz == 0.0F)
	{
		f3x3matrixAeqScalar(fR, 0.0F);
		fR[Y][Y] = 1.0F;
		if (fGp[X] >= 0.0F)
		{
			fR[X][Z] = 1.0F;
			fR[Z][X] = -1.0F;
		}
		else
		{
			fR[X][Z] = -1.0F;
			fR[Z][X] = 1.0F;
		}
		return;
	}

	// compute moduli for the general case
	fmodGyz = sqrtf(fmodGyz);
	fmodGxyz = sqrtf(fmodGxyz);
	frecipmodGxyz = 1.0F / fmodGxyz;
	ftmp = fmodGxyz / fmodGyz;

	// normalize the accelerometer reading into the z column
	for (i = X; i <= Z; i++)
	{
		fR[i][Z] = fGp[i] * frecipmodGxyz;
	}

	// construct x column of orientation matrix
	fR[X][X] = fmodGyz * frecipmodGxyz;
	fR[Y][X] = -fR[X][Z] * fR[Y][Z] * ftmp;
	fR[Z][X] = -fR[X][Z] * fR[Z][Z] * ftmp;

	// // construct y column of orientation matrix
	fR[X][Y] = 0.0F;
	fR[Y][Y] = fR[Z][Z] * ftmp;
	fR[Z][Y] = -fR[Y][Z] * ftmp;

	return;
}

// Android accelerometer 3DOF tilt function computing rotation matrix fR
void f3DOFTiltAndroid(float fR[][3], float fGp[])
{
	// the Android tilt matrix is mathematically identical to the NED tilt matrix
	// the Android self-consistency twist occurs at 90 deg roll
	f3DOFTiltNED(fR, fGp);
	return;
}

// Windows 8 accelerometer 3DOF tilt function computing rotation matrix fR
void f3DOFTiltWin8(float fR[][3], float fGp[])
{
	// the Win8 self-consistency twist occurs at 90 deg roll
	
	// local variables
	float fmodGxyz;			// modulus of the x, y, z accelerometer readings
	float fmodGxz;			// modulus of the x, z accelerometer readings
	float frecipmodGxyz;	// reciprocal of modulus
	float ftmp;				// scratch variable
	int8 i;					// counter

	// compute the accelerometer squared magnitudes
	fmodGxz = fGp[X] * fGp[X] + fGp[Z] * fGp[Z];
	fmodGxyz = fmodGxz + fGp[Y] * fGp[Y];

	// check for freefall special case where no solution is possible
	if (fmodGxyz == 0.0F)
	{
		f3x3matrixAeqI(fR);
		return;
	}

	// check for vertical up or down gimbal lock case
	if (fmodGxz == 0.0F)
	{
		f3x3matrixAeqScalar(fR, 0.0F);
		fR[X][X] = 1.0F;
		if (fGp[Y] >= 0.0F)
		{
			fR[Y][Z] = -1.0F;
			fR[Z][Y] = 1.0F;
		}
		else
		{
			fR[Y][Z] = 1.0F;
			fR[Z][Y] = -1.0F;
		}
		return;
	}

	// compute moduli for the general case
	fmodGxz = sqrtf(fmodGxz);
	fmodGxyz = sqrtf(fmodGxyz);
	frecipmodGxyz = 1.0F / fmodGxyz;
	ftmp = fmodGxyz / fmodGxz;
	if (fGp[Z] < 0.0F)
	{
		ftmp = -ftmp;
	}

	// normalize the negated accelerometer reading into the z column
	for (i = X; i <= Z; i++)
	{
		fR[i][Z] = -fGp[i] * frecipmodGxyz;
	}

	// construct x column of orientation matrix
	fR[X][X] = -fR[Z][Z] * ftmp;
	fR[Y][X] = 0.0F;
	fR[Z][X] = fR[X][Z] * ftmp;;

	// // construct y column of orientation matrix
	fR[X][Y] = fR[X][Z] * fR[Y][Z] * ftmp;
	fR[Y][Y] = -fmodGxz * frecipmodGxyz;
	if (fGp[Z] < 0.0F)
	{
		fR[Y][Y] = -fR[Y][Y];
	}	
	fR[Z][Y] = fR[Y][Z] * fR[Z][Z] * ftmp;

	return;
}

// Aerospace NED magnetometer 3DOF flat eCompass function computing rotation matrix fR
void f3DOFMagnetometerMatrixNED(float fR[][3], float fBc[])
{	
	// local variables
	float fmodBxy;			// modulus of the x, y magnetometer readings

	// compute the magnitude of the horizontal (x and y) magnetometer reading
	fmodBxy = sqrtf(fBc[X] * fBc[X] + fBc[Y] * fBc[Y]);

	// check for zero field special case where no solution is possible
	if (fmodBxy == 0.0F)
	{
		f3x3matrixAeqI(fR);
		return;
	}

	// define the fixed entries in the z row and column
	fR[Z][X] = fR[Z][Y] = fR[X][Z] = fR[Y][Z] = 0.0F;
	fR[Z][Z] = 1.0F;
	
	// define the remaining entries
	fR[X][X] = fR[Y][Y] = fBc[X] / fmodBxy;
	fR[Y][X] = fBc[Y] / fmodBxy;
	fR[X][Y] = -fR[Y][X];

	return;
}

// Android magnetometer 3DOF flat eCompass function computing rotation matrix fR
void f3DOFMagnetometerMatrixAndroid(float fR[][3], float fBc[])
{	
	// local variables
	float fmodBxy;			// modulus of the x, y magnetometer readings

	// compute the magnitude of the horizontal (x and y) magnetometer reading
	fmodBxy = sqrtf(fBc[X] * fBc[X] + fBc[Y] * fBc[Y]);

	// check for zero field special case where no solution is possible
	if (fmodBxy == 0.0F)
	{
		f3x3matrixAeqI(fR);
		return;
	}

	// define the fixed entries in the z row and column
	fR[Z][X] = fR[Z][Y] = fR[X][Z] = fR[Y][Z] = 0.0F;
	fR[Z][Z] = 1.0F;
	
	// define the remaining entries
	fR[X][X] = fR[Y][Y] = fBc[Y] / fmodBxy;
	fR[X][Y] = fBc[X] / fmodBxy;
	fR[Y][X] = -fR[X][Y];

	return;
}

// Windows 8 magnetometer 3DOF flat eCompass function computing rotation matrix fR
void f3DOFMagnetometerMatrixWin8(float fR[][3], float fBc[])
{	
	// call the Android function since it is identical to the Windows 8 matrix
	f3DOFMagnetometerMatrixAndroid(fR, fBc);

	return;
}

// NED: 6DOF e-Compass function computing rotation matrix fR
void feCompassNED(float fR[][3], float *pfDelta, float fBc[], float fGp[])
{
	// local variables
	float fmod[3];					// column moduli
	float fmodBc;					// modulus of Bc
	float fGdotBc;					// dot product of vectors G.Bc
	float ftmp;						// scratch variable
	int8 i, j;						// loop counters

	// set the inclination angle to zero in case it is not computed later
	*pfDelta = 0.0F;

	// place the un-normalized gravity and geomagnetic vectors into the rotation matrix z and x axes
	for (i = X; i <= Z; i++)
	{
		fR[i][Z] = fGp[i];
		fR[i][X] = fBc[i];
	}

	// set y vector to vector product of z and x vectors
	fR[X][Y] = fR[Y][Z] * fR[Z][X] - fR[Z][Z] * fR[Y][X];
	fR[Y][Y] = fR[Z][Z] * fR[X][X] - fR[X][Z] * fR[Z][X];
	fR[Z][Y] = fR[X][Z] * fR[Y][X] - fR[Y][Z] * fR[X][X];

	// set x vector to vector product of y and z vectors
	fR[X][X] = fR[Y][Y] * fR[Z][Z] - fR[Z][Y] * fR[Y][Z];
	fR[Y][X] = fR[Z][Y] * fR[X][Z] - fR[X][Y] * fR[Z][Z];
	fR[Z][X] = fR[X][Y] * fR[Y][Z] - fR[Y][Y] * fR[X][Z];

	// calculate the rotation matrix column moduli
	fmod[X] = sqrtf(fR[X][X] * fR[X][X] + fR[Y][X] * fR[Y][X] + fR[Z][X] * fR[Z][X]);
	fmod[Y] = sqrtf(fR[X][Y] * fR[X][Y] + fR[Y][Y] * fR[Y][Y] + fR[Z][Y] * fR[Z][Y]);
	fmod[Z] = sqrtf(fR[X][Z] * fR[X][Z] + fR[Y][Z] * fR[Y][Z] + fR[Z][Z] * fR[Z][Z]);

	// normalize the rotation matrix columns
	if (!((fmod[X] == 0.0F) || (fmod[Y] == 0.0F) || (fmod[Z] == 0.0F)))
	{
		// loop over columns j
		for (j = X; j <= Z; j++)
		{
			ftmp = 1.0F / fmod[j];
			// loop over rows i
			for (i = X; i <= Z; i++)
			{
				// normalize by the column modulus
				fR[i][j] *= ftmp;
			}
		}
	}
	else
	{
		// no solution is possible to set rotation to identity matrix
		f3x3matrixAeqI(fR);
		return;
	}

	// compute the geomagnetic inclination angle
	fmodBc = sqrtf(fBc[X] * fBc[X] + fBc[Y] * fBc[Y] + fBc[Z] * fBc[Z]);
	fGdotBc = fGp[X] * fBc[X] + fGp[Y] * fBc[Y] + fGp[Z] * fBc[Z];
	if (!((fmod[Z] == 0.0F) || (fmodBc == 0.0F)))
	{
		*pfDelta = fasin_deg(fGdotBc / (fmod[Z] * fmodBc));
	}

	return;
}

// Android: 6DOF e-Compass function computing rotation matrix fR
void feCompassAndroid(float fR[][3],  float *pfDelta, float fBc[], float fGp[])
{
	// local variables
	float fmod[3];					// column moduli
	float fmodBc;					// modulus of Bc
	float fGdotBc;					// dot product of vectors G.Bc
	float ftmp;						// scratch variable
	int8 i, j;						// loop counters

	// set the inclination angle to zero in case it is not computed later
	*pfDelta = 0.0F;

	// place the un-normalized gravity and geomagnetic vectors into the rotation matrix z and y axes
	for (i = X; i <= Z; i++)
	{
		fR[i][Z] = fGp[i];
		fR[i][Y] = fBc[i];
	}

	// set x vector to vector product of y and z vectors
	fR[X][X] = fR[Y][Y] * fR[Z][Z] - fR[Z][Y] * fR[Y][Z];
	fR[Y][X] = fR[Z][Y] * fR[X][Z] - fR[X][Y] * fR[Z][Z];
	fR[Z][X] = fR[X][Y] * fR[Y][Z] - fR[Y][Y] * fR[X][Z];

	// set y vector to vector product of z and x vectors
	fR[X][Y] = fR[Y][Z] * fR[Z][X] - fR[Z][Z] * fR[Y][X];
	fR[Y][Y] = fR[Z][Z] * fR[X][X] - fR[X][Z] * fR[Z][X];
	fR[Z][Y] = fR[X][Z] * fR[Y][X] - fR[Y][Z] * fR[X][X];

	// calculate the rotation matrix column moduli
	fmod[X] = sqrtf(fR[X][X] * fR[X][X] + fR[Y][X] * fR[Y][X] + fR[Z][X] * fR[Z][X]);
	fmod[Y] = sqrtf(fR[X][Y] * fR[X][Y] + fR[Y][Y] * fR[Y][Y] + fR[Z][Y] * fR[Z][Y]);
	fmod[Z] = sqrtf(fR[X][Z] * fR[X][Z] + fR[Y][Z] * fR[Y][Z] + fR[Z][Z] * fR[Z][Z]);

	// normalize the rotation matrix columns
	if (!((fmod[X] == 0.0F) || (fmod[Y] == 0.0F) || (fmod[Z] == 0.0F)))
	{
		// loop over columns j
		for (j = X; j <= Z; j++)
		{
			ftmp = 1.0F / fmod[j];
			// loop over rows i
			for (i = X; i <= Z; i++)
			{
				// normalize by the column modulus
				fR[i][j] *= ftmp;
			}
		}
	}
	else
	{
		// no solution is possible to set rotation to identity matrix
		f3x3matrixAeqI(fR);
		return;
	}

	// compute the geomagnetic inclination angle
	fmodBc = sqrtf(fBc[X] * fBc[X] + fBc[Y] * fBc[Y] + fBc[Z] * fBc[Z]);
	fGdotBc = fGp[X] * fBc[X] + fGp[Y] * fBc[Y] + fGp[Z] * fBc[Z];
	if (!((fmod[Z] == 0.0F) || (fmodBc == 0.0F)))
	{
		*pfDelta = fasin_deg(-fGdotBc / (fmod[Z] * fmodBc));
	}

	return;
}

// Win8: 6DOF e-Compass function computing rotation matrix fR
void feCompassWin8(float fR[][3],  float *pfDelta, float fBc[], float fGp[])
{
	// local variables
	float fmod[3];					// column moduli
	float fmodBc;					// modulus of Bc
	float fGdotBc;					// dot product of vectors G.Bc
	float ftmp;						// scratch variable
	int8 i, j;						// loop counters

	// set the inclination angle to zero in case it is not computed later
	*pfDelta = 0.0F;

	// place the negated un-normalized gravity and un-normalized geomagnetic vectors into the rotation matrix z and y axes
	for (i = X; i <= Z; i++)
	{
		fR[i][Z] = -fGp[i];
		fR[i][Y] = fBc[i];
	}

	// set x vector to vector product of y and z vectors
	fR[X][X] = fR[Y][Y] * fR[Z][Z] - fR[Z][Y] * fR[Y][Z];
	fR[Y][X] = fR[Z][Y] * fR[X][Z] - fR[X][Y] * fR[Z][Z];
	fR[Z][X] = fR[X][Y] * fR[Y][Z] - fR[Y][Y] * fR[X][Z];

	// set y vector to vector product of z and x vectors
	fR[X][Y] = fR[Y][Z] * fR[Z][X] - fR[Z][Z] * fR[Y][X];
	fR[Y][Y] = fR[Z][Z] * fR[X][X] - fR[X][Z] * fR[Z][X];
	fR[Z][Y] = fR[X][Z] * fR[Y][X] - fR[Y][Z] * fR[X][X];

	// calculate the rotation matrix column moduli
	fmod[X] = sqrtf(fR[X][X] * fR[X][X] + fR[Y][X] * fR[Y][X] + fR[Z][X] * fR[Z][X]);
	fmod[Y] = sqrtf(fR[X][Y] * fR[X][Y] + fR[Y][Y] * fR[Y][Y] + fR[Z][Y] * fR[Z][Y]);
	fmod[Z] = sqrtf(fR[X][Z] * fR[X][Z] + fR[Y][Z] * fR[Y][Z] + fR[Z][Z] * fR[Z][Z]);

	// normalize the rotation matrix columns
	if (!((fmod[X] == 0.0F) || (fmod[Y] == 0.0F) || (fmod[Z] == 0.0F)))
	{
		// loop over columns j
		for (j = X; j <= Z; j++)
		{
			ftmp = 1.0F / fmod[j];
			// loop over rows i
			for (i = X; i <= Z; i++)
			{
				// normalize by the column modulus
				fR[i][j] *= ftmp;
			}
		}
	}
	else
	{
		// no solution is possible to set rotation to identity matrix
		f3x3matrixAeqI(fR);
		return;
	}

	// compute the geomagnetic inclination angle
	fmodBc = sqrtf(fBc[X] * fBc[X] + fBc[Y] * fBc[Y] + fBc[Z] * fBc[Z]);
	fGdotBc = fGp[X] * fBc[X] + fGp[Y] * fBc[Y] + fGp[Z] * fBc[Z];
	if (!((fmod[Z] == 0.0F) || (fmodBc == 0.0F)))
	{
		*pfDelta = fasin_deg(fGdotBc / (fmod[Z] * fmodBc));
	}

	return;
}

// extract the NED angles in degrees from the NED rotation matrix
void fNEDAnglesDegFromRotationMatrix(float R[][3], float *pfPhiDeg, float *pfTheDeg, float *pfPsiDeg,
		float *pfRhoDeg, float *pfChiDeg)
{
	// calculate the pitch angle -90.0 <= Theta <= 90.0 deg
	*pfTheDeg = fasin_deg(-R[X][Z]);

	// calculate the roll angle range -180.0 <= Phi < 180.0 deg
	*pfPhiDeg = fatan2_deg(R[Y][Z], R[Z][Z]);

	// map +180 roll onto the functionally equivalent -180 deg roll
	if (*pfPhiDeg == 180.0F)
	{
		*pfPhiDeg = -180.0F;
	}

	// calculate the yaw (compass) angle 0.0 <= Psi < 360.0 deg
	if (*pfTheDeg == 90.0F)
	{
		// vertical upwards gimbal lock case
		*pfPsiDeg = fatan2_deg(R[Z][Y], R[Y][Y]) + *pfPhiDeg;
	}
	else if (*pfTheDeg == -90.0F)
	{
		// vertical downwards gimbal lock case
		*pfPsiDeg = fatan2_deg(-R[Z][Y], R[Y][Y]) - *pfPhiDeg;
	}
	else
	{
		// general case
		*pfPsiDeg = fatan2_deg(R[X][Y], R[X][X]);
	}

	// map yaw angle Psi onto range 0.0 <= Psi < 360.0 deg
	if (*pfPsiDeg < 0.0F)
	{
		*pfPsiDeg += 360.0F;
	}

	// check for rounding errors mapping small negative angle to 360 deg
	if (*pfPsiDeg >= 360.0F)
	{
		*pfPsiDeg = 0.0F;
	}

	// for NED, the compass heading Rho equals the yaw angle Psi
	*pfRhoDeg = *pfPsiDeg;

	// calculate the tilt angle from vertical Chi (0 <= Chi <= 180 deg) 
	*pfChiDeg = facos_deg(R[Z][Z]);

	return;
}

// extract the Android angles in degrees from the Android rotation matrix
void fAndroidAnglesDegFromRotationMatrix(float R[][3], float *pfPhiDeg, float *pfTheDeg, float *pfPsiDeg,
		float *pfRhoDeg, float *pfChiDeg)
{
	// calculate the roll angle -90.0 <= Phi <= 90.0 deg
	*pfPhiDeg = fasin_deg(R[X][Z]);

	// calculate the pitch angle -180.0 <= The < 180.0 deg
	*pfTheDeg = fatan2_deg(-R[Y][Z], R[Z][Z]);

	// map +180 pitch onto the functionally equivalent -180 deg pitch
	if (*pfTheDeg == 180.0F)
	{
		*pfTheDeg = -180.0F;
	}

	// calculate the yaw (compass) angle 0.0 <= Psi < 360.0 deg
	if (*pfPhiDeg == 90.0F)
	{
		// vertical downwards gimbal lock case
		*pfPsiDeg = fatan2_deg(R[Y][X], R[Y][Y]) - *pfTheDeg;
	}
	else if (*pfPhiDeg == -90.0F)
	{
		// vertical upwards gimbal lock case
		*pfPsiDeg = fatan2_deg(R[Y][X], R[Y][Y]) + *pfTheDeg;
	}
	else
	{
		// // general case
		*pfPsiDeg = fatan2_deg(-R[X][Y], R[X][X]);
	}

	// map yaw angle Psi onto range 0.0 <= Psi < 360.0 deg
	if (*pfPsiDeg < 0.0F)
	{
		*pfPsiDeg += 360.0F;
	}

	// check for rounding errors mapping small negative angle to 360 deg
	if (*pfPsiDeg >= 360.0F)
	{
		*pfPsiDeg = 0.0F;
	}

	// the compass heading angle Rho equals the yaw angle Psi
	// this definition is compliant with Motorola Xoom tablet behavior
	*pfRhoDeg = *pfPsiDeg;

	// calculate the tilt angle from vertical Chi (0 <= Chi <= 180 deg) 
	*pfChiDeg = facos_deg(R[Z][Z]);

	return;
}

// extract the Windows 8 angles in degrees from the Windows 8 rotation matrix
void fWin8AnglesDegFromRotationMatrix(float R[][3], float *pfPhiDeg, float *pfTheDeg, float *pfPsiDeg,
		float *pfRhoDeg, float *pfChiDeg)
{
	// calculate the roll angle -90.0 <= Phi <= 90.0 deg
	if (R[Z][Z] == 0.0F)
	{
		if (R[X][Z] >= 0.0F)
		{
			// tan(phi) is -infinity
			*pfPhiDeg = -90.0F;
		}
		else
		{
			// tan(phi) is +infinity
			*pfPhiDeg = 90.0F;
		}
	}
	else
	{
		// general case
		*pfPhiDeg = fatan_deg(-R[X][Z] / R[Z][Z]);
	}

	// first calculate the pitch angle The in the range -90.0 <= The <= 90.0 deg
	*pfTheDeg = fasin_deg(R[Y][Z]);

	// use R[Z][Z]=cos(Phi)*cos(The) to correct the quadrant of The remembering
	// cos(Phi) is non-negative so that cos(The) has the same sign as R[Z][Z].
	if (R[Z][Z] < 0.0F)
	{
		// wrap The around +90 deg and -90 deg giving result 90 to 270 deg
		*pfTheDeg = 180.0F - *pfTheDeg;
	}

	// map the pitch angle The to the range -180.0 <= The < 180.0 deg
	if (*pfTheDeg >= 180.0F)
	{
		*pfTheDeg -= 360.0F;
	}

	// calculate the yaw angle Psi
	if (*pfTheDeg == 90.0F)
	{
		// vertical upwards gimbal lock case: -270 <= Psi < 90 deg
		*pfPsiDeg = fatan2_deg(R[X][Y], R[X][X]) - *pfPhiDeg;
	}
	else if (*pfTheDeg == -90.0F)
	{
		// vertical downwards gimbal lock case: -270 <= Psi < 90 deg
		*pfPsiDeg = fatan2_deg(R[X][Y], R[X][X]) + *pfPhiDeg;
	}
	else
	{
		// general case: -180 <= Psi < 180 deg
		*pfPsiDeg = fatan2_deg(-R[Y][X], R[Y][Y]);

		// correct the quadrant for Psi using the value of The (deg) to give -180 <= Psi < 380 deg
		if (fabs(*pfTheDeg) >= 90.0F)
		{
			*pfPsiDeg += 180.0F;
		}
	}

	// map yaw angle Psi onto range 0.0 <= Psi < 360.0 deg
	if (*pfPsiDeg < 0.0F)
	{
		*pfPsiDeg += 360.0F;
	}

	// check for any rounding error mapping small negative angle to 360 deg
	if (*pfPsiDeg >= 360.0F)
	{
		*pfPsiDeg = 0.0F;
	}

	// compute the compass angle Rho = 360 - Psi
	*pfRhoDeg = 360.0F - *pfPsiDeg;

	// check for rounding errors mapping small negative angle to 360 deg and zero degree case
	if (*pfRhoDeg >= 360.0F)
	{
		*pfRhoDeg = 0.0F;
	}

	// calculate the tilt angle from vertical Chi (0 <= Chi <= 180 deg) 
	*pfChiDeg = facos_deg(R[Z][Z]);

	return;
}

// computes normalized rotation quaternion from a rotation vector (deg)
void fQuaternionFromRotationVectorDeg(struct fquaternion *pq, const float rvecdeg[], float fscaling)
{
	float fetadeg;			// rotation angle (deg)
	float fetarad;			// rotation angle (rad)
	float fetarad2;			// eta (rad)^2
	float fetarad4;			// eta (rad)^4
	float sinhalfeta;		// sin(eta/2)
	float fvecsq;			// q1^2+q2^2+q3^2
	float ftmp;				// scratch variable

	// compute the scaled rotation angle eta (deg) which can be both positve or negative
	fetadeg = fscaling * sqrtf(rvecdeg[X] * rvecdeg[X] + rvecdeg[Y] * rvecdeg[Y] + rvecdeg[Z] * rvecdeg[Z]);
	fetarad = fetadeg * FDEGTORAD;
	fetarad2 = fetarad * fetarad;

	// calculate the sine and cosine using small angle approximations or exact
	// angles under sqrt(0.02)=0.141 rad is 8.1 deg and 1620 deg/s (=936deg/s in 3 axes) at 200Hz and 405 deg/s at 50Hz
	if (fetarad2 <= 0.02F)
	{
		// use MacLaurin series up to and including third order
		sinhalfeta = fetarad * (0.5F - ONEOVER48 * fetarad2);
	}
	else if  (fetarad2 <= 0.06F)
	{
		// use MacLaurin series up to and including fifth order
		// angles under sqrt(0.06)=0.245 rad is 14.0 deg and 2807 deg/s (=1623deg/s in 3 axes) at 200Hz and 703 deg/s at 50Hz
		fetarad4 = fetarad2 * fetarad2;
		sinhalfeta = fetarad * (0.5F - ONEOVER48 * fetarad2 + ONEOVER3840 * fetarad4);
	}
	else
	{
		// use exact calculation
		sinhalfeta = (float)sinf(0.5F * fetarad);
	}

	// compute the vector quaternion components q1, q2, q3
	if (fetadeg != 0.0F)
	{
		// general case with non-zero rotation angle
		ftmp = fscaling * sinhalfeta / fetadeg;
		pq->q1 = rvecdeg[X] * ftmp;		// q1 = nx * sin(eta/2)
		pq->q2 = rvecdeg[Y] * ftmp;		// q2 = ny * sin(eta/2)
		pq->q3 = rvecdeg[Z] * ftmp;		// q3 = nz * sin(eta/2)
	}
	else
	{
		// zero rotation angle giving zero vector component
		pq->q1 = pq->q2 = pq->q3 = 0.0F;
	}

	// compute the scalar quaternion component q0 by explicit normalization
	// taking care to avoid rounding errors giving negative operand to sqrt
	fvecsq = pq->q1 * pq->q1 + pq->q2 * pq->q2 + pq->q3 * pq->q3;
	if (fvecsq <= 1.0F)
	{
		// normal case
		pq->q0 = sqrtf(1.0F - fvecsq);
	}
	else
	{
		// rounding errors are present
		pq->q0 = 0.0F;
	}

	return;
}

// compute the orientation quaternion from a 3x3 rotation matrix
void fQuaternionFromRotationMatrix(float R[][3], struct fquaternion *pq)
{
	float fq0sq;			// q0^2
	float recip4q0;			// 1/4q0

	// the quaternion is not explicitly normalized in this function on the assumption that it
	// is supplied with a normalized rotation matrix. if the rotation matrix is normalized then
	// the quaternion will also be normalized even if the case of small q0

	// get q0^2 and q0
	fq0sq = 0.25F * (1.0F + R[X][X] + R[Y][Y] + R[Z][Z]);
	pq->q0 = sqrtf(fabs(fq0sq));

	// normal case when q0 is not small meaning rotation angle not near 180 deg
	if (pq->q0 > SMALLQ0)
	{
		// calculate q1 to q3
		recip4q0 = 0.25F / pq->q0;
		pq->q1 = recip4q0 * (R[Y][Z] - R[Z][Y]);
		pq->q2 = recip4q0 * (R[Z][X] - R[X][Z]);
		pq->q3 = recip4q0 * (R[X][Y] - R[Y][X]);
	} // end of general case
	else
	{
		// special case of near 180 deg corresponds to nearly symmetric matrix
		// which is not numerically well conditioned for division by small q0
		// instead get absolute values of q1 to q3 from leading diagonal
		pq->q1 = sqrtf(fabs(0.5F * (1.0F + R[X][X]) - fq0sq));
		pq->q2 = sqrtf(fabs(0.5F * (1.0F + R[Y][Y]) - fq0sq));
		pq->q3 = sqrtf(fabs(0.5F * (1.0F + R[Z][Z]) - fq0sq));

		// correct the signs of q1 to q3 by examining the signs of differenced off-diagonal terms
		if ((R[Y][Z] - R[Z][Y]) < 0.0F) pq->q1 = -pq->q1;
		if ((R[Z][X] - R[X][Z]) < 0.0F) pq->q2 = -pq->q2;
		if ((R[X][Y] - R[Y][X]) < 0.0F) pq->q3 = -pq->q3;
	} // end of special case

	return;
}

// compute the rotation matrix from an orientation quaternion
void fRotationMatrixFromQuaternion(float R[][3], const struct fquaternion *pq)
{
	float f2q;
	float f2q0q0, f2q0q1, f2q0q2, f2q0q3;
	float f2q1q1, f2q1q2, f2q1q3;
	float f2q2q2, f2q2q3;
	float f2q3q3;

	// calculate products
	f2q = 2.0F * pq->q0;
	f2q0q0 = f2q * pq->q0;
	f2q0q1 = f2q * pq->q1;
	f2q0q2 = f2q * pq->q2;
	f2q0q3 = f2q * pq->q3;
	f2q = 2.0F * pq->q1;
	f2q1q1 = f2q * pq->q1;
	f2q1q2 = f2q * pq->q2;
	f2q1q3 = f2q * pq->q3;
	f2q = 2.0F * pq->q2;
	f2q2q2 = f2q * pq->q2;
	f2q2q3 = f2q * pq->q3;
	f2q3q3 = 2.0F * pq->q3 * pq->q3;

	// calculate the rotation matrix assuming the quaternion is normalized
	R[X][X] = f2q0q0 + f2q1q1 - 1.0F;
	R[X][Y] = f2q1q2 + f2q0q3;
	R[X][Z] = f2q1q3 - f2q0q2;
	R[Y][X] = f2q1q2 - f2q0q3;
	R[Y][Y] = f2q0q0 + f2q2q2 - 1.0F;
	R[Y][Z] = f2q2q3 + f2q0q1;
	R[Z][X] = f2q1q3 + f2q0q2;
	R[Z][Y] = f2q2q3 - f2q0q1;
	R[Z][Z] = f2q0q0 + f2q3q3 - 1.0F;

	return;
}

// function calculate the rotation vector from a rotation matrix
void fRotationVectorDegFromRotationMatrix(float R[][3], float rvecdeg[])
{
	float ftrace;			// trace of the rotation matrix
	float fetadeg;			// rotation angle eta (deg)
	float fmodulus;			// modulus of axis * angle vector = 2|sin(eta)|
	float ftmp;				// scratch variable

	// calculate the trace of the rotation matrix = 1+2cos(eta) in range -1 to +3
	// and eta (deg) in range 0 to 180 deg inclusive
	// checking for rounding errors that might take the trace outside this range
	ftrace = R[X][X] + R[Y][Y] + R[Z][Z];
	if (ftrace >= 3.0F)
	{
		fetadeg = 0.0F;
	}
	else if (ftrace <= -1.0F)
	{
		fetadeg = 180.0F;
	}
	else
	{
		fetadeg = acosf(0.5F * (ftrace - 1.0F)) * FRADTODEG;
	}

	// set the rvecdeg vector to differences across the diagonal = 2*n*sin(eta)
	// and calculate its modulus equal to 2|sin(eta)|
	// the modulus approaches zero near 0 and 180 deg (when sin(eta) approaches zero)
	rvecdeg[X] = R[Y][Z] - R[Z][Y];
	rvecdeg[Y] = R[Z][X] - R[X][Z];
	rvecdeg[Z] = R[X][Y] - R[Y][X];
	fmodulus = sqrtf(rvecdeg[X] * rvecdeg[X] + rvecdeg[Y] * rvecdeg[Y] + rvecdeg[Z] * rvecdeg[Z]);

	// normalize the rotation vector for general, 0 deg and 180 deg rotation cases
	if (fmodulus > SMALLMODULUS)
	{
		// general case away from 0 and 180 deg rotation
		ftmp = fetadeg / fmodulus;
		rvecdeg[X] *= ftmp;	// set x component to eta(deg) * nx
		rvecdeg[Y] *= ftmp;	// set y component to eta(deg) * ny
		rvecdeg[Z] *= ftmp;	// set z component to eta(deg) * nz
	} // end of general case
	else if (ftrace >= 0.0F)
	{
		// near 0 deg rotation (trace = 3): matrix is nearly identity matrix
		// R[Y][Z]-R[Z][Y]=2*nx*eta(rad) and similarly for other components
		ftmp = 0.5F * FRADTODEG;
		rvecdeg[X] *= ftmp;
		rvecdeg[Y] *= ftmp;
		rvecdeg[Z] *= ftmp;
	} // end of zero deg case
	else
	{
		// near 180 deg (trace = -1): matrix is nearly symmetric
		// calculate the absolute value of the components of the axis-angle vector
		rvecdeg[X] = 180.0F * sqrtf(fabs(0.5F * (R[X][X] + 1.0F)));
		rvecdeg[Y] = 180.0F * sqrtf(fabs(0.5F * (R[Y][Y] + 1.0F)));
		rvecdeg[Z] = 180.0F * sqrtf(fabs(0.5F * (R[Z][Z] + 1.0F)));

		// correct the signs of the three components by examining the signs of differenced off-diagonal terms
		if ((R[Y][Z] - R[Z][Y]) < 0.0F) rvecdeg[X] = -rvecdeg[X];
		if ((R[Z][X] - R[X][Z]) < 0.0F) rvecdeg[Y] = -rvecdeg[Y];
		if ((R[X][Y] - R[Y][X]) < 0.0F) rvecdeg[Z] = -rvecdeg[Z];
		
	} // end of 180 deg case

	return;
}

// computes rotation vector (deg) from rotation quaternion
void fRotationVectorDegFromQuaternion(struct fquaternion *pq, float rvecdeg[])
{
	float fetarad;			// rotation angle (rad)
	float fetadeg;			// rotation angle (deg)
	float sinhalfeta;		// sin(eta/2)
	float ftmp;				// scratch variable

	// calculate the rotation angle in the range 0 <= eta < 360 deg
	if ((pq->q0 >= 1.0F) || (pq->q0 <= -1.0F))
	{
		// rotation angle is 0 deg or 2*180 deg = 360 deg = 0 deg
		fetarad = 0.0F;
		fetadeg = 0.0F;
	}
	else
	{
		// general case returning 0 < eta < 360 deg 
		fetarad = 2.0F * acosf(pq->q0); 
		fetadeg = fetarad * FRADTODEG;
	}

	// map the rotation angle onto the range -180 deg <= eta < 180 deg 
	if (fetadeg >= 180.0F)
	{
		fetadeg -= 360.0F;
		fetarad = fetadeg * FDEGTORAD;
	}

	// calculate sin(eta/2) which will be in the range -1 to +1
	sinhalfeta = (float)sinf(0.5F * fetarad);

	// calculate the rotation vector (deg)
	if (sinhalfeta == 0.0F)
	{
		// the rotation angle eta is zero and the axis is irrelevant 
		rvecdeg[X] = rvecdeg[Y] = rvecdeg[Z] = 0.0F;
	}
	else
	{
		// general case with non-zero rotation angle
		ftmp = fetadeg / sinhalfeta;
		rvecdeg[X] = pq->q1 * ftmp;
		rvecdeg[Y] = pq->q2 * ftmp;
		rvecdeg[Z] = pq->q3 * ftmp;
	}

	return;
}

// function low pass filters an orientation quaternion and computes virtual gyro rotation rate
void fLPFOrientationQuaternion(struct fquaternion *pq, struct fquaternion *pLPq, float flpf, float fdeltat,
		float fOmega[], int32 loopcounter)
{
	// local variables
	struct fquaternion fdeltaq;			// delta rotation quaternion
	float rvecdeg[3];					// rotation vector (deg)
	float ftmp;							// scratch variable

	// initialize delay line on first pass: LPq[n]=q[n]
	if (loopcounter == 0)
	{
		*pLPq = *pq;
	}

	// set fdeltaqn to the delta rotation quaternion conjg(fLPq[n-1) . fqn
	fdeltaq = qconjgAxB(pLPq, pq);
	if (fdeltaq.q0 < 0.0F)
	{
		fdeltaq.q0 = -fdeltaq.q0;
		fdeltaq.q1 = -fdeltaq.q1;
		fdeltaq.q2 = -fdeltaq.q2;
		fdeltaq.q3 = -fdeltaq.q3;
	}

	// set ftmp to a scaled lpf value which equals flpf in the limit of small rotations (q0=1)
	// but which rises as the delta rotation angle increases (q0 tends to zero)
	ftmp = flpf + 0.75F * (1.0F - fdeltaq.q0);
	if (ftmp > 1.0F)
	{
		ftmp = 1.0F;
	}

	// scale the delta rotation by the corrected lpf value
	fdeltaq.q1 *= ftmp;
	fdeltaq.q2 *= ftmp;
	fdeltaq.q3 *= ftmp;

	// compute the scalar component q0
	ftmp = fdeltaq.q1 * fdeltaq.q1 + fdeltaq.q2 * fdeltaq.q2 + fdeltaq.q3 * fdeltaq.q3;
	if (ftmp <= 1.0F)
	{
		// normal case
		fdeltaq.q0 = sqrtf(1.0F - ftmp);
	}
	else
	{
		// rounding errors present so simply set scalar component to 0
		fdeltaq.q0 = 0.0F;
	}

	// calculate the delta rotation vector from fdeltaqn and the virtual gyro angular velocity (deg/s)
	fRotationVectorDegFromQuaternion(&fdeltaq, rvecdeg);
	ftmp = 1.0F / fdeltat;
	fOmega[X] = rvecdeg[X] * ftmp;
	fOmega[Y] = rvecdeg[Y] * ftmp;
	fOmega[Z] = rvecdeg[Z] * ftmp;

	// set LPq[n] = LPq[n-1] . deltaq[n]
	qAeqAxB(pLPq, &fdeltaq);

	// renormalize the low pass filtered quaternion to prevent error accumulation
	// the renormalization function ensures that q0 is non-negative
	fqAeqNormqA(pLPq);

	return;
}

// function low pass filters a scalar
void fLPFScalar(float *pfS, float *pfLPS, float flpf, int32 loopcounter)
{
	// set S[LP,n]=S[n] on first pass
	if (loopcounter == 0)
	{
		*pfLPS = *pfS;
	}

	// apply the exponential low pass filter
	*pfLPS += flpf * (*pfS - *pfLPS);

	return;
}

// function compute the quaternion product qA * qB
void qAeqBxC(struct fquaternion *pqA, const struct fquaternion *pqB, const struct fquaternion *pqC)
{
	pqA->q0 = pqB->q0 * pqC->q0 - pqB->q1 * pqC->q1 - pqB->q2 * pqC->q2 - pqB->q3 * pqC->q3;
	pqA->q1 = pqB->q0 * pqC->q1 + pqB->q1 * pqC->q0 + pqB->q2 * pqC->q3 - pqB->q3 * pqC->q2;
	pqA->q2 = pqB->q0 * pqC->q2 - pqB->q1 * pqC->q3 + pqB->q2 * pqC->q0 + pqB->q3 * pqC->q1;
	pqA->q3 = pqB->q0 * pqC->q3 + pqB->q1 * pqC->q2 - pqB->q2 * pqC->q1 + pqB->q3 * pqC->q0;

	return;
}

// function compute the quaternion product qA = qA * qB
void qAeqAxB(struct fquaternion *pqA, const struct fquaternion *pqB)
{
	struct fquaternion qProd;

	// perform the quaternion product
	qProd.q0 = pqA->q0 * pqB->q0 - pqA->q1 * pqB->q1 - pqA->q2 * pqB->q2 - pqA->q3 * pqB->q3;
	qProd.q1 = pqA->q0 * pqB->q1 + pqA->q1 * pqB->q0 + pqA->q2 * pqB->q3 - pqA->q3 * pqB->q2;
	qProd.q2 = pqA->q0 * pqB->q2 - pqA->q1 * pqB->q3 + pqA->q2 * pqB->q0 + pqA->q3 * pqB->q1;
	qProd.q3 = pqA->q0 * pqB->q3 + pqA->q1 * pqB->q2 - pqA->q2 * pqB->q1 + pqA->q3 * pqB->q0;

	// copy the result back into qA
	*pqA = qProd;

	return;
}

// function compute the quaternion product conjg(qA) * qB
struct fquaternion qconjgAxB(const struct fquaternion *pqA, const struct fquaternion *pqB)
{
	struct fquaternion qProd;

	qProd.q0 = pqA->q0 * pqB->q0 + pqA->q1 * pqB->q1 + pqA->q2 * pqB->q2 + pqA->q3 * pqB->q3;
	qProd.q1 = pqA->q0 * pqB->q1 - pqA->q1 * pqB->q0 - pqA->q2 * pqB->q3 + pqA->q3 * pqB->q2;
	qProd.q2 = pqA->q0 * pqB->q2 + pqA->q1 * pqB->q3 - pqA->q2 * pqB->q0 - pqA->q3 * pqB->q1;
	qProd.q3 = pqA->q0 * pqB->q3 - pqA->q1 * pqB->q2 + pqA->q2 * pqB->q1 - pqA->q3 * pqB->q0;

	return qProd;
}

// function normalizes a rotation quaternion and ensures q0 is non-negative
void fqAeqNormqA(struct fquaternion *pqA)
{
	float fNorm;					// quaternion Norm

	// calculate the quaternion Norm
	fNorm = sqrtf(pqA->q0 * pqA->q0 + pqA->q1 * pqA->q1 + pqA->q2 * pqA->q2 + pqA->q3 * pqA->q3);
	if (fNorm > CORRUPTQUAT)
	{
		// general case
		fNorm = 1.0F / fNorm;
		pqA->q0 *= fNorm;
		pqA->q1 *= fNorm;
		pqA->q2 *= fNorm;
		pqA->q3 *= fNorm;
	}
	else
	{
		// return with identity quaternion since the quaternion is corrupted
		pqA->q0 = 1.0F;
		pqA->q1 = pqA->q2 = pqA->q3 = 0.0F;
	}

	// correct a negative scalar component if the function was called with negative q0
	if (pqA->q0 < 0.0F)
	{
		pqA->q0 = -pqA->q0;
		pqA->q1 = -pqA->q1;
		pqA->q2 = -pqA->q2;
		pqA->q3 = -pqA->q3;
	}

	return;
}

// set a quaternion to the unit quaternion
void fqAeq1(struct fquaternion *pqA)
{
	pqA->q0 = 1.0F;
	pqA->q1 = pqA->q2 = pqA->q3 = 0.0F;

	return;
}
