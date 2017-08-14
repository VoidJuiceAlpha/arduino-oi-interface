/* begin oi_io_synch.h */
/*
 *  synchronous input functions
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
#include "oi_simple.h"

//--------------------------------------------------------------------------
// Synchronous Serial Input
//--------------------------------------------------------------------------

// Low level synchronous reception of sensor byte
uint8_t oi_GetByte(void);


//--------------------------------------------------------------------------
// Synchronous Serial Input Queue Empty
//--------------------------------------------------------------------------

// Synchronous input queue empty
void oi_EmptyByteQueue(void);

//--------------------------------------------------------------------------
//  Sensor Input Reception Functions
//--------------------------------------------------------------------------
// Note: These calls require prior serial port initialization using
//       the oi_SerialSynchInputInitialize() function call

// Synchronous request and return of a sensor packet
void oi_SensorReturnSynch(uint8_t sensor_packetID, uint8_t *packet_buffer); 

// Synchronous request and return of multiple sensor packets
void oi_SensorReturnSynchList(uint8_t *packet_buffer, uint8_t num_packets, ... ); 

// Synchronous return of a previously requested sensor packet 
// Use this function for obtaining input from SensorRequest requests
// and for collecting arbitrary length input from the device
void oi_SensorReturn(uint8_t sensor_packet_size, volatile uint8_t *packet_buffer);

/* end oi_io_synch.h */
