/* begin oi_io_asynch.h */
/*
 *  asynchronous sensor input routines
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

// Include header files
#include <avr/interrupt.h>
#include <avr/io.h>

#include "oi_simple.h"
#include "oi_timer_asynch.h"

#define MAX_PACKET_SIZE 52
//--------------------------------------------------------------------------
// Individual Asynchronous Sensor Input Functions
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Asynchronous Sensor Input Request
//--------------------------------------------------------------------------

// Asynchronous request for sensor data
void 
oi_SensorRequestAsynch(	uint8_t sensor_packetID);

//--------------------------------------------------------------------------
//  Sensor Input Data Collection Functions
//--------------------------------------------------------------------------
// Collect asynchronously generated sensor data
uint8_t 
oi_SensorReturnAsynch(uint8_t sensor_packet_ID, 
					  uint8_t *packet_buffer);

// Delay for the specified time in ms and update all the sensor values.
// No other requests for sensor input should be made outside this routine.
// Uncollected sensor bytes may be left in the input buffer if 
// the timer expires before transmission of the entire packet is complete.
void 
oi_SensorUpdateAndDelay(uint8_t sensor_packet_ID, 
						uint8_t *packet_buffer, 
						uint16_t *distance, uint16_t *angle, 
						uint16_t time_ms);


/* end oi_io_asynch.h */
