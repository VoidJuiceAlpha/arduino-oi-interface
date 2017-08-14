/* begin oi_simple.c  - macro version */
/*
 *  oi_simple Command Interface
 *  for both iRobot Create and also Roomba Cleaning devices
 *  --- Macro Version ---
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
  
/*
 *  If you are intending to use the oi_simple interface for Roomba or similar 
 *  cleaning devices only, rather than the Create, then #define OI_ROOMBA 
 *  before any use of oi_simple.h macros and functions
 *
 *  example include: 
 *		#define OI_ROOMBA
 *		  // include of oi_simple 
 *		#include "oi_simple.h"
 */
 
// Includes
#include <stdarg.h>			// for variable arguments
#include "oi_simple.h"


// global array for sensor input packet sizes needs to live somewhere
// and here is a good place
oi_DeclarePacketSizeArray();

//--------------------------------------------------------------------------
// Song Related Commands
//-------------------------------------------------------------------------- 
void 
oi_SongStoreList(uint8_t song_slot, uint8_t num_notes, ... ) 
{ 
  va_list marker;
  uint8_t i;
  
	oi_PutByte(OI_CMD_SONG); 
	oi_PutByte(song_slot); 
	oi_PutByte(num_notes); 
	va_start(marker, num_notes);
	for (i=0; i < num_notes; i++) {
	  oi_PutByte( va_arg(marker, int) );
	  oi_PutByte( va_arg(marker, int) );
	  }
	va_end(marker);
}
  
//--------------------------------------------------------------------------
// Sensor Data Commands
//--------------------------------------------------------------------------
void 
oi_SensorRequestList(uint8_t num_packets, ... ) 
{ 
  va_list marker;
  uint8_t i;
  
	oi_PutByte(OI_CMD_SENSORLIST); 
	oi_PutByte(num_packets); 
	va_start(marker, num_packets);
	for (i=0; i < num_packets; i++) {
	  oi_PutByte( va_arg(marker, int) );
	  }
	va_end(marker);
}
  
//Stream Commands
void 
oi_StreamList(uint8_t num_packets, ... ) 
{ 
  va_list marker;
  uint8_t i;
  
	oi_PutByte(OI_CMD_STREAM); 
	oi_PutByte(num_packets); 
	va_start(marker, num_packets);
	for (i=0; i < num_packets; i++) {
	  oi_PutByte( va_arg(marker, int) );
	  }
	va_end(marker);
}
  
 /* end oi_simple.c  - macro version */
