/* begin oi_simple.h */
/*
 *  iRobot Open Interface oi_simple Command Interface
 *  for both Create and also Roomba Cleaning devices
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

#define COMMAND_MODULE

// Includes
#include "oi_output_synch.h"

#define true  1
#define false 0

//--------------------------------------------------------------------------
// Maximum Size of a Sensor Return Packet 
//--------------------------------------------------------------------------
#define MAX_PACKET_SIZE 52

//--------------------------------------------------------------------------
// Sensor Packet Size Array
//--------------------------------------------------------------------------
extern const uint8_t SenPacketSize[43];

// defined in oi_simple.c compilation module
// must be defined and accessible for input routines
#define oi_DeclarePacketSizeArray() \
const uint8_t SenPacketSize[43] = { \
	26, 10,  6, 10, 14, 12, 52,  1,  1,  1, \
	 1,  1,  1,  1,  1,  1,  1,  1,  1,  2, \
	 2,  1,  2,  2,  1,  2,  2,  2,  2,  2, \
	 2,  2,  1,  2,  1,  1,  1,  1,  1,  2, \
	 2,  2,  2 \
	} 

//----------------------------------------------------------------------------------------
// OI Command Values
//----------------------------------------------------------------------------------------
		// the commands below are shared by both Roomba cleaning devices and Create device
		//--------------------------------------------------------------------------------
#define OI_CMD_START        		128
#define OI_CMD_BAUD         		129
#define OI_CMD_CONTROL      		130
#define OI_CMD_SAFE         		131
#define OI_CMD_FULL         		132
				// POWER turn off command not available in Create
#define OI_CMD_POWER				133
#define OI_CMD_SPOT         		134
#define OI_CMD_COVER	       		135
#define OI_CMD_MAXDEMO         		136
#define OI_CMD_DRIVE        		137
				// calls PWM version of low side drivers command
#define OI_CMD_MOTORS       		138
#define OI_CMD_LEDS         		139
#define OI_CMD_SONG         		140
#define OI_CMD_PLAYSONG     		141
#define OI_CMD_SENSORS      		142
#define OI_CMD_COVERANDDOCK         143
		// the commands below are exposed in the Create Interface
		//--------------------------------------------------------------------------------
#define OI_CMD_PWMMOTORS    		144
#define OI_CMD_DRIVEWHEELS  		145
#define OI_CMD_UNSPECIFIED		 	146
#define OI_CMD_OUTPUTS      		147
#define OI_CMD_STREAM				148
#define OI_CMD_SENSORLIST			149
#define OI_CMD_STREAMPAUSERESUME	150
#define OI_CMD_IRCHAR       		151
#define OI_CMD_SCRIPT       		152
#define OI_CMD_SCRIPTPLAY   		153
#define OI_CMD_SCRIPTSHOW   		154
#define OI_CMD_WAIT					155
#define OI_CMD_WAITDISTANCE			156
#define OI_CMD_WAITANGLE			157
#define OI_CMD_WAITEVENT			158


//----------------------------------------------------------------------------------------
// Baud Code Values
//----------------------------------------------------------------------------------------
#define BAUD_300         0
#define BAUD_600         1
#define BAUD_1200        2
#define BAUD_2400        3
#define BAUD_4800        4
#define BAUD_9600        5
#define BAUD_14400       6
#define BAUD_19200       7
#define BAUD_28800       8
#define BAUD_38400       9
#define BAUD_57600       10
#define BAUD_115200      11

#define LED_OFF	0
#define LED_ON	1

//--------------------------------------------------------------------------
// LED State Input Constant Values
//--------------------------------------------------------------------------
#define LED_OFF	0
#define LED_ON	1

//--------------------------------------------------------------------------
// Drive Radius Special Cases Constant Values
//--------------------------------------------------------------------------
#define RadStraight     32768
#define RadCCW          1
#define RadCW           -1

//--------------------------------------------------------------------------
// Sensor Related Constant Values
//--------------------------------------------------------------------------
// Sensor packet IDs
#define SEN_PKT_Combo0       0  /* packets 7-26  */
#define SEN_PKT_Combo1       1  /* packets 7-16  */
#define SEN_PKT_Combo2       2  /* packets 17-20 */
#define SEN_PKT_Combo3       3  /* packets 21-26 */
#define SEN_PKT_Combo4       4  /* packets 27-34 */
#define SEN_PKT_Combo5       5  /* packets 35-42 */


#define SEN_PKT_ComboAll     6	/* all packets   */
#define SEN_PKT_BumpDrop     7
#define SEN_PKT_Wall         8
#define SEN_PKT_CliffL       9
#define SEN_PKT_CliffFL      10
#define SEN_PKT_CliffFR      11
#define SEN_PKT_CliffR       12
#define SEN_PKT_VWall        13
#define SEN_PKT_OverC        14
#ifdef ROOMBA
#define SEN_PKT_DirtL        15
#define SEN_PKT_DirtR        16
#endif
#define SEN_PKT_IRChar       17
#define SEN_PKT_Button       18
#define SEN_PKT_Dist         19
#define SEN_PKT_Angle        20
#define SEN_PKT_ChargeState  21
#define SEN_PKT_Volt         22
#define SEN_PKT_Curr         23
#define SEN_PKT_Temp         24
#define SEN_PKT_Charge       25
#define SEN_PKT_Cap          26
#define SEN_PKT_WallSig      27
#define SEN_PKT_CliffLSig    28
#define SEN_PKT_CliffFLSig   29
#define SEN_PKT_CliffFRSig   30
#define SEN_PKT_CliffRSig    31
#define SEN_PKT_Inputs       32
#define SEN_PKT_AInput       33
#define SEN_PKT_ChAvailable  34
#define SEN_PKT_OIMode       35
#define SEN_PKT_OISong       36
#define SEN_PKT_OISongPlay   37
#define SEN_PKT_StreamPckts  38
#define SEN_PKT_Vel          39
#define SEN_PKT_Rad          40
#define SEN_PKT_VelR         41
#define SEN_PKT_VelL         42

// Sensor byte indices - offsets in packets 0, 5 and 6
#define SEN_OFFSET_BumpDrop     0            
#define SEN_OFFSET_Wall         1
#define SEN_OFFSET_CliffL       2
#define SEN_OFFSET_CliffFL      3
#define SEN_OFFSET_CliffFR      4
#define SEN_OFFSET_CliffR       5
#define SEN_OFFSET_VWall        6
#define SEN_OFFSET_OverC        7
#define SEN_OFFSET_IRChar       10
#define SEN_OFFSET_Button       11
#define SEN_OFFSET_Dist1        12
#define SEN_OFFSET_Dist0        13
#define SEN_OFFSET_Ang1         14
#define SEN_OFFSET_Ang0         15
#define SEN_OFFSET_ChargeState  16
#define SEN_OFFSET_Volt1        17
#define SEN_OFFSET_Volt0        18
#define SEN_OFFSET_Curr1        19
#define SEN_OFFSET_Curr0        20
#define SEN_OFFSET_Temp         21
#define SEN_OFFSET_Charge1      22
#define SEN_OFFSET_Charge0      23
#define SEN_OFFSET_Cap1         24
#define SEN_OFFSET_Cap0         25
#define SEN_OFFSET_WallSig1     26
#define SEN_OFFSET_WallSig0     27
#define SEN_OFFSET_CliffLSig1   28
#define SEN_OFFSET_CliffLSig0   29
#define SEN_OFFSET_CliffFLSig1  30
#define SEN_OFFSET_CliffFLSig0  31
#define SEN_OFFSET_CliffFRSig1  32
#define SEN_OFFSET_CliffFRSig0  33
#define SEN_OFFSET_CliffRSig1   34
#define SEN_OFFSET_CliffRSig0   35
#define SEN_OFFSET_Inputs       36
#define SEN_OFFSET_AInput1      37
#define SEN_OFFSET_AInput0      38
#define SEN_OFFSET_ChAvailable  39
#define SEN_OFFSET_OIMode       40
#define SEN_OFFSET_OISong       41
#define SEN_OFFSET_OISongPlay   42
#define SEN_OFFSET_StreamPckts  43
#define SEN_OFFSET_Vel1         44
#define SEN_OFFSET_Vel0         45
#define SEN_OFFSET_Rad1         46
#define SEN_OFFSET_Rad0         47
#define SEN_OFFSET_VelR1        48
#define SEN_OFFSET_VelR0        49
#define SEN_OFFSET_VelL1        50
#define SEN_OFFSET_VelL0        51

// Sensor bit masks
#define SEN_MASK_WheelDropFront  0x10
#define SEN_MASK_WheelDropLeft   0x08
#define SEN_MASK_WheelDropRight  0x04
#define SEN_MASK_BumpLeft        0x02
#define SEN_MASK_BumpRight       0x01
#define SEN_MASK_BumpBoth        0x03
#define SEN_MASK_BumpEither      0x03
#define SEN_MASK_WheelDropAll    0x1C
#define SEN_MASK_ButtonAdvance   0x04
#define SEN_MASK_ButtonPlay      0x01



//--------------------------------------------------------------------------
// Mode Commands
//--------------------------------------------------------------------------
void oi_ModeStart(void); 
void oi_ModeControl(void); /* deprecated */
void oi_ModeSafe(void); 	
void oi_ModeFull(void); 	
void oi_ModePower(void); 

//--------------------------------------------------------------------------
// Baudrate Command
//--------------------------------------------------------------------------

// Sets the baud rate of the Roomba/Create device.
void oi_BaudRate(uint8_t rate); 

//--------------------------------------------------------------------------
// Demo and Cleaning Commands
//--------------------------------------------------------------------------
void oi_Demo(uint8_t demo_slot); 
void oi_MaxClean(uint8_t demo_slot); 
void oi_Spot(void); 
void oi_Cover(void); 
void oi_CoverAndDock(void); 

//--------------------------------------------------------------------------
// Drive and Motor Commands
//--------------------------------------------------------------------------
void oi_Drive(uint16_t velocity, uint16_t radius); 
void oi_DriveDirect(uint16_t right_velocity, uint16_t left_velocity); 	
  // Low Side Motor Drivers Control
void oi_Motors(uint8_t lowside_0, uint8_t lowside_1, uint8_t lowside_2); 
void oi_MotorsPWM(uint8_t driver0, uint8_t driver1, uint8_t driver2); 

//--------------------------------------------------------------------------
// LED State Control Commands for Roomba and Create
//--------------------------------------------------------------------------
#ifndef OI_ROOMBA 
#define oi_LEDSet oi_LEDState_create
#elif
#define oi_LEDSet oi_LEDState_roomba
#endif
void oi_LEDState_create(uint8_t advance, uint8_t play, uint8_t power_color, 
                        uint8_t power_intensity); 
void oi_LEDState_roomba(uint8_t status, uint8_t spot, uint8_t clean, 
                        uint8_t max, uint8_t dirt_detect, uint8_t power_color, 
					    uint8_t power_intensity);
						
//--------------------------------------------------------------------------
// Song Related Commands
//--------------------------------------------------------------------------
	// A song is created by a call to oi_SongCreate_hdr followed by
	// the appropriate number of calls to oi_SongCreate_Note_data
void oi_SongStoreList(uint8_t song_slot, uint8_t num_notes, ... ); 
void oi_SongPlay(uint8_t song_slot); 

//--------------------------------------------------------------------------
// Sensor Data Commands
//--------------------------------------------------------------------------
// Get Sensor Data Commands
void oi_SensorRequest(uint8_t sensor_packet);
void oi_SensorRequestList(uint8_t num_packets, ... ); 

//Stream Commands
void oi_StreamList(uint8_t num_packets, ... ); 
void oi_StreamPause(void); 
void oi_StreamResume(void); 

//--------------------------------------------------------------------------
// Digital Output Commands
//--------------------------------------------------------------------------
void oi_DigitalOutput(uint8_t output0, uint8_t output1, uint8_t output2); 

// Send IR Character Out of Low Side Driver 1 Control
void oi_SendIR(char char2); 

//--------------------------------------------------------------------------
// Script Commands
//--------------------------------------------------------------------------
void oi_Script_hdr(uint8_t num_command_bytes);
void oi_ScriptPlay(void); 
void oi_ScriptShow(void); 

//--------------------------------------------------------------------------
// Wait Commands
// (intended for use in scripts)
//--------------------------------------------------------------------------
	// wait values are in terms of 15ms units
void oi_Wait(uint8_t wait_value);
void oi_WaitDistance(uint16_t wait_value); 
void oi_WaitDistance_byte(uint8_t wait_high, uint8_t wait_low);
void oi_WaitAngle(uint16_t wait_value); 
void oi_WaitAngle_byte(uint8_t wait_high, uint8_t wait_low); 
void oi_WaitEvent(int8_t event); 

 /* end oi_simple.h */
