/* begin oi_timer_asynch.c */
/*
 *  asynchronous timer functions
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

// Include header files 
#include <avr/interrupt.h>
#include <avr/io.h>

#include "oi_simple.h"

// countdown timer state; used internally
volatile uint16_t oi_timer_count = 0;
// flag pointer variable; used internally; points to user defined flag variable
volatile uint8_t *oi_timer_on = 0;

//--------------------------------------------------------------------------
//  Asynchronous Timer Functions
//--------------------------------------------------------------------------
void
oi_TimerAsynchInitialize(void)
{
#ifdef COMMAND_MODULE
  cli();
    // Set up timer 1 to generate an interrupt every 1 ms
  TCCR1A = 0x00;
  TCCR1B = (_BV(WGM12) | _BV(CS12));
  OCR1A = 71;
  TIMSK1 = _BV(OCIE1A);
  sei();
#endif
}

void 
oi_TimerAsynchRequest(uint16_t time_ms, uint8_t *flag)
{
  oi_timer_count = time_ms;  
  oi_timer_on = flag;
  *oi_timer_on = true;
}

void 
oi_Timer(uint16_t time_ms)
{
  uint8_t flag;
  oi_timer_count = time_ms;  
  oi_timer_on = &flag;
  *oi_timer_on = true;
  while(*oi_timer_on);
}

// Command Module AVR Timer 1 interrupt to time delays in ms
// uses timer_on to indicate it is still decrementing time count
// uses timer_cnt as the time countdown timer value
#ifdef COMMAND_MODULE
SIGNAL(SIG_OUTPUT_COMPARE1A)
{
	// if countdown not complete, decrement counter 
	// and leave oi_timer_on still on
  if(oi_timer_count)
    oi_timer_count--;
	// otherwise counter expired, so turn timer_on flag off
  else
    *oi_timer_on = false;
}
#endif


/* end oi_timer_asynch.c */
