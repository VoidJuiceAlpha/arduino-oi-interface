/* begin oi_output_synch.c */
/*
 *  synchronous output routines
 *  for
 *  oi_simple Command Interface
 *  for both iRobot Create and also Roomba Cleaning devices
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

#define COMMAND_MODULE

// Include header files
#include <avr/io.h>
#include <avr/interrupt.h>

#include "oi_simple.h"

#ifndef OI_TIMER_ASYNCH
#include "oi_timer_synch.h"
#elif
#include "oi_timer_asynch.h"
#endif

#ifdef COMMAND_MODULE
// Atmel Baud UBRRx values
#define Ubrr300         3839
#define Ubrr600         1919
#define Ubrr1200        959
#define Ubrr2400        479
#define Ubrr4800        239
#define Ubrr9600        119
#define Ubrr14400       79
#define Ubrr19200       59
#define Ubrr28800       39
#define Ubrr38400       29
#define Ubrr57600       19
#define Ubrr115200      9
#endif

//--------------------------------------------------------------------------
// Serial Port Initialization
//--------------------------------------------------------------------------
// Set up the controller platform serial port 
// for asynchronous or synchronous rx interrupts
// as well as synchronous output
void
oi_SerialPortInitialize(uint8_t asynchronous_input)
{  
  if (asynchronous_input){
      // asynch input & synch output
#ifdef COMMAND_MODULE
      // Disable interrupts
    cli();  
    UBRR0 = 19; // 57600 baud
    UCSR0B = (_BV(RXCIE0) | _BV(TXEN0) | _BV(RXEN0));
    UCSR0C = (_BV(UCSZ00) | _BV(UCSZ01));
      // Enable interrupts
    sei();
#endif
    }
  else { 
      // synch input & synch output
#ifdef COMMAND_MODULE
      // Disable interrupts
    cli();  
      // Set up the serial port for 57600 baud (synchronous)
    UBRR0 = 19; // 57600 baud
    UCSR0B = (_BV(TXEN0) | _BV(RXEN0));
    UCSR0C = (_BV(UCSZ00) | _BV(UCSZ01));
      //  Interrupts stay disabled
//    sei();
#endif
    }
}

//--------------------------------------------------------------------------
// Synchronous Serial Output
//--------------------------------------------------------------------------

// Transmit a byte over the serial port
void oi_PutByte(uint8_t value)
{
#ifdef COMMAND_MODULE
	// block while transmit busy
  while(!(UCSR0A & _BV(UDRE0))) ;
    // transmit this byte
  UDR0 = value;
#endif
}

// Transmission over the serial port in process
uint8_t 
oi_PutByte_Busy(void)
{
#ifdef COMMAND_MODULE
  return (!(UCSR0A & _BV(TXC0))) ;
#endif
}

//--------------------------------------------------------------------------
// Set the Baud Rate
//--------------------------------------------------------------------------

// Controller Baud Rate Control
void 
oi_BaudRatePlatform(uint8_t baud_code)
{
  if(baud_code <= BAUD_115200) {
  
#ifdef COMMAND_MODULE

    cli();

      // Switch the baud rate register
    if(baud_code == BAUD_115200)
      UBRR0 = Ubrr115200;
    else if(baud_code == BAUD_57600)
      UBRR0 = Ubrr57600;
    else if(baud_code == BAUD_38400)
      UBRR0 = Ubrr38400;
    else if(baud_code == BAUD_28800)
      UBRR0 = Ubrr28800;
    else if(baud_code == BAUD_19200)
      UBRR0 = Ubrr19200;
    else if(baud_code == BAUD_14400)
      UBRR0 = Ubrr14400;
    else if(baud_code == BAUD_9600)
      UBRR0 = Ubrr9600;
    else if(baud_code == BAUD_4800)
      UBRR0 = Ubrr4800;
    else if(baud_code == BAUD_2400)
      UBRR0 = Ubrr2400;
    else if(baud_code == BAUD_1200)
      UBRR0 = Ubrr1200;
    else if(baud_code == BAUD_600)
      UBRR0 = Ubrr600;
    else if(baud_code == BAUD_300)
      UBRR0 = Ubrr300;

    sei();
#endif

    oi_Timer(100);
    }
}

// Baud Rate Control for both Controller and Roomba/Create
void 
oi_BaudRateAll(uint8_t rate)
{
    // Send the Create/Roomba baud change command for 28800 baud
  oi_BaudRate(BAUD_28800);

#ifdef COMMAND_MODULE
  UCSR0A |= _BV(TXC0);
#endif
 
    // Wait while the command is finished being sent
  while(oi_PutByte_Busy());
  
    // Change the controller's baud rate
  oi_BaudRatePlatform(BAUD_28800);
}

/* end oi_output_synch.c */
