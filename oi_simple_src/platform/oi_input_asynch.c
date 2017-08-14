/* begin oi_input_asynch.c */
/*
 *  asynchronous sensor input routines
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
 
#define COMMAND_MODULE

// Include header files
#include <avr/interrupt.h>
#include <avr/io.h>

#include "oi_simple.h"
#include "oi_timer_asynch.h"

//--------------------------------------------------------------------------
// asynchronous interupt routine variables
//--------------------------------------------------------------------------

volatile uint8_t sensors_in[MAX_PACKET_SIZE];		// direct sensor input buffer
volatile uint8_t sensors_index = 0;					// current index into sensor input buffer 
volatile uint8_t sensors_input_pending = false;		// true if input request is in process
uint8_t sensor_packet_size = 0;						// size of currently requested packet


//--------------------------------------------------------------------------
// Asynchronous Input Interrupt Routine
//--------------------------------------------------------------------------

// Asynchronous serial receive interrupt routine to store sensor values
// Depends on proper buffer target setup values from oi_SensorRequestAsynch()
#ifdef COMMAND_MODULE
SIGNAL(SIG_USART_RECV)
{
  uint8_t temp;
    // put current byte in temp
  temp = UDR0;		
    // store byte temp into sensor input buffer 
	// *only* if expecting more input
	// otherwise throw it away
  if(sensors_input_pending) {  
	  // store current byte in buffer and increment index
    sensors_in[sensors_index++] = temp;
	  // turn off sensors_flag if buffer is filled with sensor_packet_size bytes
    if(sensors_index >= sensor_packet_size)
      sensors_input_pending = false;
    }
}
#endif

//--------------------------------------------------------------------------
// Asynchronous Sensor Input Request
//--------------------------------------------------------------------------

// Asynchronous request for sensor data
void 
oi_SensorRequestAsynch(uint8_t sensor_packetID) 
{
    // set up for asynchronous packet reception
  	// request packet 
  oi_PutByte(OI_CMD_SENSORS); 
  oi_PutByte(sensor_packetID); 
    // set up for reception
  sensors_index = 0;
  sensor_packet_size = SenPacketSize[sensor_packetID];
  sensors_input_pending = true; // ready for incoming packet data
}

//--------------------------------------------------------------------------
//  Sensor Input Data Collection Functions
//--------------------------------------------------------------------------
// Collect asynchronously generated sensor data
uint8_t 
oi_SensorReturnAsynch(uint8_t sensor_packet_ID, 
					  uint8_t *packet_buffer)
{
  if(!sensors_input_pending) {
    uint8_t index;	 // index temp for transfer of data from buffer to array
		// last packet requested is done, 
		// so move it from input buffer into requested buffer
    for(index = 0; index < sensor_packet_size; index++)
      packet_buffer[index] = sensors_in[index];
	return true;
	}
  else return false;
}

// Delay for the specified time in ms and update all the sensor values
void 
oi_SensorUpdateAndDelay(uint8_t sensor_packet_ID, 
						uint8_t *packet_buffer, 
						uint16_t *distance, uint16_t *angle, 
						uint16_t time_ms)
{
  uint8_t timer_on;					// true if timer is still running
  uint8_t sensor_packet_size;		// size of requested packet
    // get size of packet
  sensor_packet_size = SenPacketSize[sensor_packet_ID];
    // initialize timer 
  oi_TimerAsynchRequest(time_ms,&timer_on);
	// busy wait loop until timer decrements down to 0
  while(timer_on) {
	  // while waiting check for sensor input and get it
    if(oi_SensorReturnAsynch(sensor_packet_ID, packet_buffer)) {
        // Update running totals of distance and angle
		// done each time a full sensor packet is received even while waiting
      *distance += (int)((packet_buffer[SEN_OFFSET_Dist1] << 8) | packet_buffer[SEN_OFFSET_Dist0]);
      *angle += (int)((packet_buffer[SEN_OFFSET_Ang1] << 8) | packet_buffer[SEN_OFFSET_Ang0]);
	    // Set up for and request asynchronous reception of sensor data
      oi_SensorRequestAsynch(sensor_packet_ID);
      }
    }
}

/* end oi_input_asynch.c */
