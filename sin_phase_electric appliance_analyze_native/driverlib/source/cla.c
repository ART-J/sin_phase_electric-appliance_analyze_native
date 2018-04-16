//###########################################################################
//
// FILE:   cla.c
//
// TITLE:  CLA Driver Implementation File
//
//###########################################################################
// $TI Release: F2837xD Support Library v3.02.00.00 $
// $Release Date: Sat Sep 16 15:29:00 CDT 2017 $
// $Copyright:
// Copyright (C) 2013-2017 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

#include "cla.h"

//*****************************************************************************
//
// CLA_setTriggerSource()
//
//*****************************************************************************
void
CLA_setTriggerSource(CLA_TaskNumber taskNumber, CLA_Trigger trigger)
{
    uint32_t srcSelReg;
    uint32_t shiftVal;

    //
    // Calculate the shift value for the specified task.
    //
    shiftVal = ((uint32_t)taskNumber * SYSCTL_CLA1TASKSRCSEL1_TASK2_S) % 32U;

    //
    // Calculate the register address for the specified task.
    //
    if(taskNumber <= CLA_TASK_4)
    {
        //
        // Tasks 1-4
        //
        srcSelReg = (uint32_t)DMACLASRCSEL_BASE + SYSCTL_O_CLA1TASKSRCSEL1;
    }
    else
    {
        //
        // Tasks 5-8
        //
        srcSelReg = (uint32_t)DMACLASRCSEL_BASE + SYSCTL_O_CLA1TASKSRCSEL2;
    }

    EALLOW;

    //
    // Write trigger selection to the appropriate register.
    //
    HWREG(srcSelReg) &= ~((uint32_t)SYSCTL_CLA1TASKSRCSEL1_TASK1_M
                           << shiftVal);
    HWREG(srcSelReg) = HWREG(srcSelReg) | ((uint32_t)trigger << shiftVal);

    EDIS;
}