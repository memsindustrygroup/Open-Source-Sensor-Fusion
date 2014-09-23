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
#ifndef EVENTS_H
#define EVENTS_H

#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "MQX1.h"
#include "SystemTimer1.h"
#include "LED_RED.h"
#include "LED_GREEN.h"
#include "LED_BLUE.h"
#include "FTM.h"
#include "UART.h"
#include "I2C.h"
#include "TestPin_KF_Time.h"
#include "TestPin_MagCal_Time.h"

// function prototypes for Event.c
void Cpu_OnNMIINT(void);
void FTM_OnCounterRestart(LDD_TUserData *UserDataPtr);
void UART_OnBlockSent(LDD_TUserData *UserDataPtr);
void I2C_OnMasterBlockSent(LDD_TUserData *UserDataPtr);
void I2C_OnMasterBlockReceived(LDD_TUserData *UserDataPtr);
void I2C_OnError(LDD_TUserData *UserDataPtr);
void UART_OnTxComplete(LDD_TUserData *UserDataPtr);
void UART_OnBlockReceived(LDD_TUserData *UserDataPtr);

#endif // #ifndef EVENTS_H
