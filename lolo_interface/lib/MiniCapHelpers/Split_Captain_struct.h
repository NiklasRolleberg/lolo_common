#pragma once
#include "helper.h"
#include <stdint.h>
#include <stddef.h>  // Required for offsetof macro

#define CAPTAINID  3
#define CapdtoEMSID 31
#define EMSdFCUtoCapID 13
#define EMSdSYStoCapID 14

/* ------------------ Data Structures ------------------ */

typedef struct {
// FCU Data
  float ANODE_PRESSURE;
    float CATHODE_PRESSURE;
    float COOLANT_VALVE_POSITION;
    float O2_HUMIDITY;
    float H2_HUMIDITY;
    float H2_FRESH_PRESSURE;
    float O2_FRESH_PRESSURE;
    float ANODE_TEMPERATURE;
    float CATHODE_TEMPERATURE;
    float O2_FRESH_TEMP;
    float H2_FRESH_TEMP;
    float O2_TEMP;
    float H2_TEMP;
    float STACK_CURRENT;
    float STACK_VOLTAGE;
    float BAT_VOLTAGE;
    float MIN_CELL_VOLTAGE;
    float MAX_CELL_VOLTAGE;
    float AVG_CELL_VOLTAGE;
    float H2_CONCENTRATION;
    float MONITOR_SAFETY_STATE;
    float SAFETY_FAULT;
    float STARTUP_STATUS;
    float SHUTDOWN_STATUS;
    float FEEDBACK_CONTROL_SETPOINT;
    float STACK_CURRENT_SETPOINT;
    float AIR_PUMP_KNF_SETPOINT;
    float OXYGEN_CUTOFF_SETPOINT;
    float H2_PUMP_KNF_SETPOINT;
    float COOLANT_PUMP_SETPOINT;
    float H2_FRESH_SETPOINT;
    float CONTACTOR_SETPOINT;
    float PURGE_VALVE_SETPOINT;
    float O2_FRESH_SETPOINT;
    float POWER_CYCLE_DCDC_CVM_SETPOINT;
    float SAFETY_STATE;
    uint64_t SAFETY_ACTIVE_MASK;      // Which errors are active NOW
    uint64_t SAFETY_LATCHED_MASK;	  // Which errors have/had been active since boot
    uint8_t  SAFETY_ACTIVE_COUNT;     // How many errors active NOW
    uint32_t SAFETY_TOTAL_COUNT;      // Total error events since boot
    uint32_t MS_SINCE_BOOT;
} CaptainFCURxData_t;



typedef struct {
		//BMU DATA
		float BMU_CELL_VOLTAGE[8];
		float BMU_BATTERY_VOLTAGE;
		float BMU_BATTERY_CURRENT;
		float BMU_SOC;
		uint16_t BMU_STATUS;
		float BMU_TEMP_BMS_BOARD;
		float BMU_TEMP_SENSOR_1;
		float BMU_TEMP_SENSOR_2;
		uint8_t BMU_MAIN_SWITCH;
		uint8_t BMU_ALLOW_ON;
		uint8_t BMU_LOAD_ENABLE;
		uint8_t BMU_CHARGE_ENABLE;
		float BMU_SUPPLY_VOLTAGE;
		float BMU_CURRENT;
		float BMU_PRESSURE;
		uint8_t BMU_LEAK_DETECTED;
		uint64_t BMU_TIMESTAMP;
		//EMS data
		float EMS_FCU_current;
		float EMS_Load_current;
        uint8_t EMS_SYSTEM_STATE;	
		// 6 LSB encode (CAPtx,CAPrx,BMStx,BMSrx,FCUtx,FCUrx)
        uint8_t EMS_COMMS_STATUS;
		float EMS_POWER_SETPOINT;
        float EMS_H2_CONSUMPTION;
        float EMS_O2_CONSUMPTION;
    	float EMS_STACK_POWER;
		float EMS_FCU_POWER;
		float EMS_LOAD_POWER;
		uint32_t txKernelCount;				//TX watchdog kernel count
} CaptainSystemRxData_t;


/* ------------------ Field Descriptors ------------------ */

FieldDescriptor_t captainFCURxDataFields[] = {
	// New fields specific to Captain communication
	//FCU
	{"anodePressure",              F_FLOAT, offsetof(CaptainFCURxData_t, ANODE_PRESSURE),                1},
	{"cathodePressure",            F_FLOAT, offsetof(CaptainFCURxData_t, CATHODE_PRESSURE),              1},
	{"coolantValvePosition",       F_FLOAT, offsetof(CaptainFCURxData_t, COOLANT_VALVE_POSITION),        1},
	{"o2Humidity",                 F_FLOAT, offsetof(CaptainFCURxData_t, O2_HUMIDITY),                   1},
	{"h2Humidity",                 F_FLOAT, offsetof(CaptainFCURxData_t, H2_HUMIDITY),                   1},
	{"h2FreshPressure",            F_FLOAT, offsetof(CaptainFCURxData_t, H2_FRESH_PRESSURE),             1},
	{"o2FreshPressure",            F_FLOAT, offsetof(CaptainFCURxData_t, O2_FRESH_PRESSURE),             1},
	{"anodeTemperature",           F_FLOAT, offsetof(CaptainFCURxData_t, ANODE_TEMPERATURE),             1},
	{"cathodeTemperature",         F_FLOAT, offsetof(CaptainFCURxData_t, CATHODE_TEMPERATURE),           1},
	{"o2FreshTemp",                F_FLOAT, offsetof(CaptainFCURxData_t, O2_FRESH_TEMP),                 1},
	{"h2FreshTemp",                F_FLOAT, offsetof(CaptainFCURxData_t, H2_FRESH_TEMP),                 1},
	{"o2Temp",                     F_FLOAT, offsetof(CaptainFCURxData_t, O2_TEMP),                       1},
	{"h2Temp",                     F_FLOAT, offsetof(CaptainFCURxData_t, H2_TEMP),                       1},
	{"stackCurrent",               F_FLOAT, offsetof(CaptainFCURxData_t, STACK_CURRENT),                 1},
	{"stackVoltage",               F_FLOAT, offsetof(CaptainFCURxData_t, STACK_VOLTAGE),                 1},
	{"batteryVoltage",             F_FLOAT, offsetof(CaptainFCURxData_t, BAT_VOLTAGE),                   1},
	{"minCellVoltage",             F_FLOAT, offsetof(CaptainFCURxData_t, MIN_CELL_VOLTAGE),              1},
	{"maxCellVoltage",             F_FLOAT, offsetof(CaptainFCURxData_t, MAX_CELL_VOLTAGE),              1},
	{"avgCellVoltage",             F_FLOAT, offsetof(CaptainFCURxData_t, AVG_CELL_VOLTAGE),              1},
	{"h2Concentration",            F_FLOAT, offsetof(CaptainFCURxData_t, H2_CONCENTRATION),              1},
	{"monitorSafetyState",         F_FLOAT, offsetof(CaptainFCURxData_t, MONITOR_SAFETY_STATE),          1},
	{"safetyFault",                F_FLOAT, offsetof(CaptainFCURxData_t, SAFETY_FAULT),                  1},
	{"startupState",               F_FLOAT, offsetof(CaptainFCURxData_t, STARTUP_STATUS),                1},
	{"shutdownState",              F_FLOAT, offsetof(CaptainFCURxData_t, SHUTDOWN_STATUS),               1},
	{"feedbackControlSetpoint",    F_FLOAT, offsetof(CaptainFCURxData_t, FEEDBACK_CONTROL_SETPOINT),     1},
	{"stackCurrentSetpoint",       F_FLOAT, offsetof(CaptainFCURxData_t, STACK_CURRENT_SETPOINT),        1},
	{"airPumpKNFSetpoint",         F_FLOAT, offsetof(CaptainFCURxData_t, AIR_PUMP_KNF_SETPOINT),         1},
	{"oxygenCutoffSetpoint",       F_FLOAT, offsetof(CaptainFCURxData_t, OXYGEN_CUTOFF_SETPOINT),        1},
	{"h2PumpKNFSetpoint",          F_FLOAT, offsetof(CaptainFCURxData_t, H2_PUMP_KNF_SETPOINT),          1},
	{"coolantPumpSetpoint",        F_FLOAT, offsetof(CaptainFCURxData_t, COOLANT_PUMP_SETPOINT),         1},
	{"h2FreshSetpoint",            F_FLOAT, offsetof(CaptainFCURxData_t, H2_FRESH_SETPOINT),             1},
	{"contactorSetpoint",          F_FLOAT, offsetof(CaptainFCURxData_t, CONTACTOR_SETPOINT),            1},
	{"purgeValveSetpoint",         F_FLOAT, offsetof(CaptainFCURxData_t, PURGE_VALVE_SETPOINT),          1},
	{"o2FreshSetpoint",            F_FLOAT, offsetof(CaptainFCURxData_t, O2_FRESH_SETPOINT),             1},
	{"powerCycleDcdcCvmSetpoint",  F_FLOAT, offsetof(CaptainFCURxData_t, POWER_CYCLE_DCDC_CVM_SETPOINT), 1},
	{"safety_state",               F_FLOAT, offsetof(CaptainFCURxData_t, SAFETY_STATE),                  1},
	
    {"safetyActiveMask",  		 F_UINT64, offsetof(CaptainFCURxData_t, SAFETY_ACTIVE_MASK),   		1},
	{"safetyLatchedMask",   	 F_UINT64, offsetof(CaptainFCURxData_t, SAFETY_LATCHED_MASK),   	1},
    {"safetyActiveCount",  		 F_UINT8,  offsetof(CaptainFCURxData_t, SAFETY_ACTIVE_COUNT),  		1},
    {"safetyTotalCount",   		 F_UINT32, offsetof(CaptainFCURxData_t, SAFETY_TOTAL_COUNT),   		1}, 
	{"msSinceBoot",   				 F_UINT32, offsetof(CaptainFCURxData_t, MS_SINCE_BOOT),   				1},
	};
	
	FieldDescriptor_t captainSystemRxDataFields[] = {
		//BMU
	{"cellVoltage",       F_FLOAT,  offsetof(CaptainSystemRxData_t, BMU_CELL_VOLTAGE),       	8},
    {"bmubatteryVoltage",    F_FLOAT,  offsetof(CaptainSystemRxData_t, BMU_BATTERY_VOLTAGE),    	1},
    {"batteryCurrent",    F_FLOAT,  offsetof(CaptainSystemRxData_t, BMU_BATTERY_CURRENT),    	1},
    {"SOC",               F_FLOAT,  offsetof(CaptainSystemRxData_t, BMU_SOC),               	1},
    {"status",            F_UINT16, offsetof(CaptainSystemRxData_t, BMU_STATUS),            	1},
    {"tempBMSBoard",      F_FLOAT,  offsetof(CaptainSystemRxData_t, BMU_TEMP_BMS_BOARD),      1},
    {"tempSensor1",       F_FLOAT,  offsetof(CaptainSystemRxData_t, BMU_TEMP_SENSOR_1),       1},
    {"tempSensor2",       F_FLOAT,  offsetof(CaptainSystemRxData_t, BMU_TEMP_SENSOR_2),       1},
    {"mainSwitch",        F_UINT8,  offsetof(CaptainSystemRxData_t, BMU_MAIN_SWITCH),        	1},
    {"allowOn",           F_UINT8,  offsetof(CaptainSystemRxData_t, BMU_ALLOW_ON),           	1},
    {"loadEnable",        F_UINT8,  offsetof(CaptainSystemRxData_t, BMU_LOAD_ENABLE),        	1},
    {"chargeEnable",      F_UINT8,  offsetof(CaptainSystemRxData_t, BMU_CHARGE_ENABLE),     	1},
    {"supplyVoltageBMU",  F_FLOAT,  offsetof(CaptainSystemRxData_t, BMU_SUPPLY_VOLTAGE),  		1},
    {"currentBMU",        F_FLOAT,  offsetof(CaptainSystemRxData_t, BMU_CURRENT),        			1},
    {"pressure",          F_FLOAT,  offsetof(CaptainSystemRxData_t, BMU_PRESSURE),          	1},
    {"leakDetected",      F_UINT8,  offsetof(CaptainSystemRxData_t, BMU_LEAK_DETECTED),      	1},
	{"ts",                F_UINT64, offsetof(CaptainSystemRxData_t, BMU_TIMESTAMP),           1},

		//EMS
	{"emsFCUCurrent",     F_FLOAT, offsetof(CaptainSystemRxData_t, EMS_FCU_current),			1},
	{"emsLoadCurrent",    F_FLOAT, offsetof(CaptainSystemRxData_t, EMS_Load_current),      1},
	{"systemState",       F_UINT8, offsetof(CaptainSystemRxData_t, EMS_SYSTEM_STATE), 1},
    {"emsCommsStatus",    F_UINT8,  offsetof(CaptainSystemRxData_t, EMS_COMMS_STATUS),           	1},
	{"powerSetpoint",     F_FLOAT, offsetof(CaptainSystemRxData_t, EMS_POWER_SETPOINT),       1},
    {"h2Consumption",     F_FLOAT, offsetof(CaptainSystemRxData_t, EMS_H2_CONSUMPTION),       1},
    {"o2Consumption",     F_FLOAT, offsetof(CaptainSystemRxData_t, EMS_O2_CONSUMPTION),       1},
    {"stackPower",     	  F_FLOAT, offsetof(CaptainSystemRxData_t, EMS_STACK_POWER),       1},
    {"fcuPower",     	  F_FLOAT, offsetof(CaptainSystemRxData_t, EMS_FCU_POWER),       1},
	{"loadPower",     	  F_FLOAT, offsetof(CaptainSystemRxData_t, EMS_LOAD_POWER),       1},
	{"txKernelCount",     F_UINT32, offsetof(CaptainSystemRxData_t, txKernelCount),   				1}
};

#define CAPTAIN_FCURXDATA_FIELD_COUNT (sizeof(captainFCURxDataFields)/sizeof(captainFCURxDataFields[0]))
#define CAPTAIN_SYSTEMRXDATA_FIELD_COUNT (sizeof(captainSystemRxDataFields)/sizeof(captainSystemRxDataFields[0]))

