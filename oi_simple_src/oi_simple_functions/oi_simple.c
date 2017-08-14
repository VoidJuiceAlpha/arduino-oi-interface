/* begin oi_simple.c */
/*
 *  oi_simple Command Interface
 *  for both iRobot Create and also Roomba Cleaning devices
 *  --- Function Version ---
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
// Mode Commands
//--------------------------------------------------------------------------
void 
oi_ModeStart(void) 
{ 
	oi_PutByte(OI_CMD_START); 
}

/* Control Mode is deprecated */
void 
oi_ModeControl(void) 
{ 
    oi_PutByte(OI_CMD_CONTROL); 
}


void 
oi_ModeSafe(void) 
{ 
	oi_PutByte(OI_CMD_SAFE); 
}
	
void 
oi_ModeFull(void) 
{ 
	oi_PutByte(OI_CMD_FULL); 
}
	
#ifdef OI_ROOMBA 
#define oi_ModePower(void) 
{ 
	oi_PutByte(OI_CMD_POWER); 
}
#endif

//--------------------------------------------------------------------------
// Baudrate Command
//--------------------------------------------------------------------------
// Sets the baud rate of the Roomba/Create device.
void 
oi_BaudRate(uint8_t rate) 
{ 
    oi_PutByte(OI_CMD_BAUD); 
    oi_PutByte(rate); 
}

//--------------------------------------------------------------------------
// Demo and Cleaning Commands
//--------------------------------------------------------------------------
#ifndef OI_ROOMBA 
void 
oi_Demo(uint8_t demo_slot) 
{ 
	oi_PutByte(OI_CMD_MAXDEMO); 
	oi_PutByte(demo_slot); 
} 
#endif  

#ifdef OI_ROOMBA 
void 
oi_MaxClean(uint8_t demo_slot) 
{ 
	oi_PutByte(OI_CMD_MAXDEMO); 
	oi_PutByte(demo_slot); 
}
#endif   

void 
oi_Spot(void) 
{ 
	oi_PutByte(OI_CMD_SPOT); 
}

void 
oi_Cover(void) 
{ 
	oi_PutByte(OI_CMD_COVER); 
}

void 
oi_CoverAndDock(void) 
{ 
	oi_PutByte(OI_CMD_COVERANDDOCK); 
}

//--------------------------------------------------------------------------
// Drive and Motor Commands
//--------------------------------------------------------------------------
void 
oi_Drive(uint16_t velocity, uint16_t radius) 
{  
	oi_PutByte(OI_CMD_DRIVE); 
	oi_PutByte((uint8_t) (((velocity) >> 8) & 0x00FF)); 
	oi_PutByte((uint8_t) ((velocity) & 0x00FF)); 
	oi_PutByte((uint8_t) (((radius) >> 8) & 0x00FF)); 
	oi_PutByte((uint8_t) ((radius) & 0x00FF)); 
}   

void 
oi_DriveDirect(uint16_t right_velocity, uint16_t left_velocity) 
{  
	oi_PutByte(OI_CMD_DRIVEWHEELS); 
	oi_PutByte((uint8_t) (((right_velocity) >> 8) & 0x00FF)); 
	oi_PutByte((uint8_t) ((right_velocity) & 0x00FF)); 
	oi_PutByte((uint8_t) (((left_velocity) >> 8) & 0x00FF)); 
	oi_PutByte((uint8_t) ((left_velocity) & 0x00FF)); 
}

  // Low Side Motor Drivers Control
void oi_Motors(uint8_t lowside_0, uint8_t lowside_1, uint8_t lowside_2) 
{  
	oi_PutByte(OI_CMD_MOTORS); 
	oi_PutByte((lowside_2 << 2) | (lowside_1 << 1)| lowside_0 ); 
}

void 
oi_MotorsPWM(uint8_t lowside_0, uint8_t lowside_1, uint8_t lowside_2) 
{ 
	oi_PutByte(OI_CMD_PWMMOTORS); 
	oi_PutByte(lowside_2); 
	oi_PutByte(lowside_1); 
	oi_PutByte(lowside_0); 
}

//--------------------------------------------------------------------------
// LED State Control Commands for Roomba and Create
//--------------------------------------------------------------------------
#ifndef OI_ROOMBA 
void 
oi_LEDState_create(uint8_t advance, uint8_t play, uint8_t power_color, uint8_t power_intensity) 
{ 
	oi_PutByte(OI_CMD_LEDS); 
	oi_PutByte((advance << 3) | (play << 1)); 
	oi_PutByte((uint8_t) power_color); 
	oi_PutByte((uint8_t) power_intensity); 
}
#elif
void 
oi_LEDState_roomba(uint8_t status, uint8_t spot, uint8_t clean, uint8_t max, uint8_t dirt_detect, uint8_t power_color, uint8_t power_intensity) 
{ 
	oi_PutByte(OI_CMD_LEDS); 
	oi_PutByte((status << 4) | (spot << 3) | (clean << 2) | (max << 1) | dirt_detect); 
	oi_PutByte(power_color); 
	oi_PutByte(power_intensity); 
}
#endif

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
  
void 
oi_SongPlay(uint8_t song_slot) 
{ 
	oi_PutByte(OI_CMD_PLAYSONG); 
	oi_PutByte(song_slot); 
}

//--------------------------------------------------------------------------
// Sensor Data Commands
//--------------------------------------------------------------------------
void 
oi_SensorRequest(uint8_t sensor_packet) 
{ 
	oi_PutByte(OI_CMD_SENSORS); 
	oi_PutByte(sensor_packet); 
}

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
  
void 
oi_StreamPause(void) 
{ 
	oi_PutByte(OI_CMD_STREAMPAUSERESUME); 
	oi_PutByte(0); 
}

void 
oi_StreamResume(void) 
{ 
	oi_PutByte(OI_CMD_STREAMPAUSERESUME); 
	oi_PutByte(1); 
}

//--------------------------------------------------------------------------
// Digital Output Commands
//--------------------------------------------------------------------------
void 
oi_DigitalOutput(uint8_t output0, uint8_t output1, uint8_t output2) 
{ 
	oi_PutByte(OI_CMD_OUTPUTS); 
	oi_PutByte((output0 << 2) | (output1 << 1) | (output2)); 
}

// Send IR Character Out of Low Side Driver 1 Control
void 
oi_SendIR(char char2) 
{ 
	oi_PutByte(OI_CMD_IRCHAR); 
	oi_PutByte(char2); 
}

//--------------------------------------------------------------------------
// Script Commands
//--------------------------------------------------------------------------
void 
oi_Script_hdr(uint8_t num_command_bytes) 
{ 
	oi_PutByte(OI_CMD_SCRIPT); 
	oi_PutByte(num_command_bytes); 
}

void 
oi_ScriptPlay(void) 
{ 
	oi_PutByte(OI_CMD_SCRIPTPLAY); 
}

void 
oi_ScriptShow() 
{ 
	oi_PutByte(OI_CMD_SCRIPTSHOW); 
}

//--------------------------------------------------------------------------
// Wait Commands
// (intended for use in scripts)
//--------------------------------------------------------------------------
	// wait values are in terms of 15ms units
void 
oi_Wait(uint8_t wait_value) 
{ 
	oi_PutByte(OI_CMD_WAIT); 
	oi_PutByte(wait_value); 
}	

void 
oi_WaitDistance(uint16_t wait_value) 
{ 
	oi_PutByte(OI_CMD_WAITDISTANCE);
	oi_PutByte((wait_value >> 8) & 0x00FF); 
	oi_PutByte(wait_value & 0x00FF); 
}

void 
oi_WaitDistance_byte(uint8_t wait_high, uint8_t wait_low) 
{ 
	oi_PutByte(OI_CMD_WAITDISTANCE); 
	oi_PutByte(wait_high); 
	oi_PutByte(wait_low); 
}

void 
oi_WaitAngle(uint16_t wait_value) 
{ 
	oi_PutByte(OI_CMD_WAITANGLE); 
	oi_PutByte((wait_value >> 8) & 0x00FF); 
	oi_PutByte(wait_value & 0x00FF); 
}

void oi_WaitAngle_byte(uint8_t wait_high, uint8_t wait_low) 
{ 
	oi_PutByte(OI_CMD_WAITANGLE); 
	oi_PutByte(wait_high); 
	oi_PutByte(wait_low); 
}

void oi_WaitEvent(int8_t event) 
{ 
	oi_PutByte(OI_CMD_WAITEVENT); 
	oi_PutByte((uint8_t) event); 
}


 /* end oi_simple.c */
