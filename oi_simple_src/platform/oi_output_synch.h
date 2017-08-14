/* begin oi_output_synch.h */
/*
 *  synchronous output routines
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

#include <avr/io.h>

//--------------------------------------------------------------------------
// Serial Port Initialization
//--------------------------------------------------------------------------

// Set up the controller platform serial port 
// for asynchronous or synchronous rx interrupts
// as well as synchronous output
void
oi_SerialPortInitialize(uint8_t asynchronous_input);

//--------------------------------------------------------------------------
// Synchronous Serial Output
//--------------------------------------------------------------------------

// Transmit a byte over the serial port
void oi_PutByte(uint8_t value);

// Transmission over the serial port in process
uint8_t oi_PutByte_Busy(void);

//--------------------------------------------------------------------------
// Set the Baud Rate
//--------------------------------------------------------------------------

// Controller Baud Rate Control
void oi_BaudRatePlatform(uint8_t baud_rate);

// Baud Rate Control for both Controller and Roomba/Create
void oi_BaudRateAll(uint8_t baud_rate);


/* end oi_output_synch.h */
