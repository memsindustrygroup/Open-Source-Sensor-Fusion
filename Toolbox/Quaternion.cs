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

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace SensorFusion
{
    public static class Quaternion
    {
        // general constants
        private const int X = 0;
        private const int Y = 1;
        private const int Z = 2;

        // quaternion structure definition
        public struct fquaternion
        {
            public double q0;	// scalar component
            public double q1;	// x vector component
            public double q2;	// y vector component
            public double q3;	// z vector component
        };

        // function compute the quaternion product qA * qB
        public static fquaternion qAxB(fquaternion qA, fquaternion qB)
        {
            fquaternion qProd;

            qProd.q0 = qA.q0 * qB.q0 - qA.q1 * qB.q1 - qA.q2 * qB.q2 - qA.q3 * qB.q3;
            qProd.q1 = qA.q0 * qB.q1 + qA.q1 * qB.q0 + qA.q2 * qB.q3 - qA.q3 * qB.q2;
            qProd.q2 = qA.q0 * qB.q2 - qA.q1 * qB.q3 + qA.q2 * qB.q0 + qA.q3 * qB.q1;
            qProd.q3 = qA.q0 * qB.q3 + qA.q1 * qB.q2 - qA.q2 * qB.q1 + qA.q3 * qB.q0;

            return qProd;
        }

        // function compute the quaternion product conjg(qA) * qB
        public static fquaternion qconjgAxB(fquaternion qA, fquaternion qB)
        {
            fquaternion qProd;

            qProd.q0 = qA.q0 * qB.q0 + qA.q1 * qB.q1 + qA.q2 * qB.q2 + qA.q3 * qB.q3;
            qProd.q1 = qA.q0 * qB.q1 - qA.q1 * qB.q0 - qA.q2 * qB.q3 + qA.q3 * qB.q2;
            qProd.q2 = qA.q0 * qB.q2 + qA.q1 * qB.q3 - qA.q2 * qB.q0 - qA.q3 * qB.q1;
            qProd.q3 = qA.q0 * qB.q3 - qA.q1 * qB.q2 + qA.q2 * qB.q1 - qA.q3 * qB.q0;

            return qProd;
        }

        // function computes the rotation matrix from an orientation quaternion
        public static void fRotationMatrixFromQuaternion(ref double[,] R, ref fquaternion q)
        {
            double f2q;
            double f2q0q0, f2q0q1, f2q0q2, f2q0q3;
            double f2q1q1, f2q1q2, f2q1q3;
            double f2q2q2, f2q2q3;
            double f2q3q3;

            // calculate products
            f2q = 2.0F * q.q0;
            f2q0q0 = f2q * q.q0;
            f2q0q1 = f2q * q.q1;
            f2q0q2 = f2q * q.q2;
            f2q0q3 = f2q * q.q3;
            f2q = 2.0F * q.q1;
            f2q1q1 = f2q * q.q1;
            f2q1q2 = f2q * q.q2;
            f2q1q3 = f2q * q.q3;
            f2q = 2.0F * q.q2;
            f2q2q2 = f2q * q.q2;
            f2q2q3 = f2q * q.q3;
            f2q3q3 = 2.0F * q.q3 * q.q3;

            // calculate the rotation matrix assuming the quaternion is normalized
            R[X, X] = f2q0q0 + f2q1q1 - 1.0F;
            R[X, Y] = f2q1q2 + f2q0q3;
            R[X, Z] = f2q1q3 - f2q0q2;
            R[Y, X] = f2q1q2 - f2q0q3;
            R[Y, Y] = f2q0q0 + f2q2q2 - 1.0F;
            R[Y, Z] = f2q2q3 + f2q0q1;
            R[Z, X] = f2q1q3 + f2q0q2;
            R[Z, Y] = f2q2q3 - f2q0q1;
            R[Z, Z] = f2q0q0 + f2q3q3 - 1.0F;

            return;
        }
    }
}
