// cmndreader.h
//----------------

// code that defines specific commands for aArtisan

// *** BSD License ***
// ------------------------------------------------------------------------------------------
// Copyright (c) 2011, MLG Properties, LLC
// All rights reserved.
//
// Contributor:  Jim Gallt
//
// Redistribution and use in source and binary forms, with or without modification, are
// permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this list of
//   conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this list
//   of conditions and the following disclaimer in the documentation and/or other materials
//   provided with the distribution.
//
//   Neither the name of the copyright holder(s) nor the names of its contributors may be
//   used to endorse or promote products derived from this software without specific prior
//   written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ------------------------------------------------------------------------------------------

// Version 1.10

#ifndef CMNDREADER_H
#define CMNDREADER_H

#include "cmndproc.h"
#include <ModbusIP_ESP8266.h>

// ----------------------- commands
#define PID_CMD "PID" // turn PID ON or OFF
#define IO3_CMD "IO3" // 0 to 100 percent PWM 5V output on IO3
#define OT1_CMD "OT1" // 0 to 100 percent PWM 5V output on IO3

// forward declarations
class pidCmnd;
class io3Cmnd;
class ot1Cmnd;

// external declarations of class objects

extern ModbusIP mb;
extern pidCmnd pid;
extern io3Cmnd io3;
extern ot1Cmnd ot1;


// extern declarations for functions, variables in the main program
extern bool pid_on_status;



// class declarations for commands

class pidCmnd : public CmndBase
{
public:
    pidCmnd();
    virtual boolean doCommand(CmndParser *pars);
};

class io3Cmnd : public CmndBase
{
public:
    io3Cmnd();
    virtual boolean doCommand(CmndParser *pars);
};

class ot1Cmnd : public CmndBase
{
public:
    ot1Cmnd();
    virtual boolean doCommand(CmndParser *pars);
};
#endif
