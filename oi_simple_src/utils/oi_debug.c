/* begin oi_debug.c */
/*
 *  Debug Tools
 *  for both Create and also Roomba Cleaning devices
 *
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
 
#include <avr/io.h>
#include <stdlib.h>
#include "oi_timer_asynch.h"
 
#define LED1              0x20
#define LED1Off           (PORTD |= LED1)
#define LED1On            (PORTD &= ~LED1)

#define LED2              0x40
#define LED2Off           (PORTD |= LED2)
#define LED2On            (PORTD &= ~LED2)

#define LEDBoth           0x60
#define LEDBothOff        (PORTD |= LEDBoth)
#define LEDBothOn         (PORTD &= ~LEDBoth)

//--------------------------------------------------------------------------
// Debug routine flashes given number
//--------------------------------------------------------------------------
void 
oi_Debug(uint8_t number) 
{   
  LEDBothOff;	
  oi_Timer(700);
  
  if (number == 0) LEDBothOff;
  else if (number == 1) LED1On;
  else if (number == 2) LED2On;
  else if (number == 3) LEDBothOn;

  oi_Timer(1000);
}

void 
oi_DebugFlasher(uint8_t number) 
{ 
  uint8_t i;
  
  LEDBothOn;	// Create LEDs on
  oi_Timer(1000);
  
  for (i = number; i > 0; i--) {
    LEDBothOff;	// Create LEDs off
	oi_Timer(100);
    LED1On;	// Create LEDs on
	oi_Timer(200);
	}

  LEDBothOn;	// Create LEDs on
  oi_Timer(1000);
}

 /* end oi_debug.c */
