/* begin oi_power.c */
/*
 *  power up routines
 *  for
 *  oi_simple Command Interface
 *  for both iRobot Create and also Roomba Cleaning devices
 */
 
  /* Copyright (c) 2007, Kevin Weiler
     All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 
*/

//--------------------------------------------------------------------------
// Command Module / Create Power Control Ports
//--------------------------------------------------------------------------
#define COMMAND_MODULE

#include <avr/io.h>

#ifdef COMMAND_MODULE
// Create Port
#define RobotPwrToggle      0x80
#define RobotPwrToggleHigh (PORTD |= 0x80)
#define RobotPwrToggleLow  (PORTD &= ~0x80)

#define RobotPowerSense    0x20
#define RobotIsOn          (PINB & RobotPowerSense)
#endif

//--------------------------------------------------------------------------
// Power for the Create
//--------------------------------------------------------------------------

#ifndef OI_TIMER_ASYNCH
#include "oi_timer_synch.h"
#elif
#include "oi_timer_asynch.h"
#endif

void 
oi_PowerOn(void)
{
    // If Create's power is off, turn it on
  if(!RobotIsOn) {
      while(!RobotIsOn) {
          RobotPwrToggleLow;
          oi_Timer(500);       // Delay in this state
          RobotPwrToggleHigh;  // Low to high transition to toggle power
          oi_Timer(100);       // Delay in this state
          RobotPwrToggleLow;
          }
      oi_Timer(3500);  // Delay for startup
      }
}

void 
oi_PowerOff(void)
{
    // If Create's power is on, turn it off
	
  /* to be implemented */
  
}


/* end oi_power.c */
