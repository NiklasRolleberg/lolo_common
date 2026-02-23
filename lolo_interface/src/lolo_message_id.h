/*! \file scientistmsg.h    
    This file contains package IDs for the lolo interface.
*/

#ifndef LOLO_MESSAGE_IDS
#define LOLO_MESSAGE_IDS

//V1.4
//2025 02 13

//LOLO -> ROS2
#define CS_LEAK             7
#define CS_STATUS           8
#define CS_CONTROL          9
#define CS_RUDDER           10
#define CS_ELEVATOR         12
#define CS_THRUSTER_PORT    13
#define CS_THRUSTER_STRB    14

#define CS_EMSdFCUtoCapID 30
#define CS_EMSdSYStoCapID 31

#define CS_VTHRUSTER_1      61
#define CS_VTHRUSTER_2      62
#define CS_VTHRUSTER_3      63
#define CS_VTHRUSTER_4      64

#define CS_BATTERY1         15
#define CS_BATTERY2         55
#define CS_TEMP             16 //Internal temperatures
#define CS_BARO             17 //Internal pressures

#define CS_ELEVON_STRB      21
#define CS_ELEVON_PORT      22
#define CS_USBL_RECEIVED    41
#define CS_SATELITE_RECEIVED 43
#define CS_MENUSTREAM       50

//ROS2 -> LOLO
#define SC_HEARTBEAT             150
#define SC_ABORT                 151
#define SC_DROP_WEIGHT           153

#define SC_SET_ELEVON_PORT       159
#define SC_SET_ELEVON_STRB       160
#define SC_SET_RUDDER            161
#define SC_SET_ELEVATOR          162
#define SC_SET_THRUSTER_PORT     163
#define SC_SET_THRUSTER_STRB     164
#define SC_SET_VTHRUSTER_1      61
#define SC_SET_VTHRUSTER_2      62
#define SC_SET_VTHRUSTER_3      63
#define SC_SET_VTHRUSTER_4      64

#define SC_MENUSTREAM            200

#define SC_NAV_DATA             110
#define SC_ALTITUDE_DATA        111

#define SC_USBL_TRANSMIT        142
#define SC_SATELITE_TRANSMIT    143

#define SC_SETTINGS 170
#define SC_SETTINGS_CONTROL_SOURCE_NONE 1
#define SC_SETTINGS_CONTROL_SOURCE_MANUAL_CONTROL 2
#define SC_SETTINGS_CONTROL_SOURCE_ONBOARD_CONTROL 3
#define SC_SETTINGS_CONTROL_SOURCE_EXTERNAL_CONRTOL 4
#define SC_SETTINGS_CONTROL_SOURCE_EMERGENCY_CONTROL 5
#define SC_SETTINGS_CONTROL_MODE_STANDBY 6
#define SC_SETTINGS_CONTROL_MODE_DISARMED 7
#define SC_SETTINGS_CONTROL_MODE_ARMED 8
#define SC_SETTINGS_THRUSTERS_ENABLED 9
#define SC_SETTINGS_THRUSTERS_DISABLED 10
#define SC_SETTINGS_VERTICAL_THRUSTERS_ENABLED 11
#define SC_SETTINGS_VERTICAL_THRUSTERS_DISABLED 12

#endif