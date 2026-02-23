#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "../udpinterface/CaptainInterFace.h"

typedef enum {
    F_UINT8,
    F_UINT16,
    F_UINT32,
    F_UINT64,
    F_FLOAT
} FieldType_t;

typedef struct {
    const char* name;    // optional, for debugging
    FieldType_t type;    // how to encode
    size_t offset;       // offset in the data blob
    size_t count;        // array size, 1 for scalar
} FieldDescriptor_t;

typedef enum {
	SAFE_NONE = 0,
	SAFE_H2_OVER_4VOL, //4% VOL --> 40000 ppm
	SAFE_H2_3TO4VOL,
	SAFE_H2_1TO3VOL,
	SAFE_H2_SENSOR_ERROR,
	SAFE_H2_SENSOR_COMM_LOST,
	SAFE_O2_VESSEL_OVER_1P1,	//PRESSURE INSIDE PRESSURE VESSEL?? MEASURE ON DUTY CYCLE OF SOLENOID TODO
	SAFE_TEMP_TOO_HIGH,
	SAFE_TEMP_HIGH,
	SAFE_O2_PUMP_LOST,
	SAFE_H2_PUMP_LOST,
	SAFE_COOL_PUMP_LOST,
	SAFE_COOL_VALVE_LOST,
	SAFE_CVM_COMMUNICATION_LOST,
	SAFE_DCDC_COMMUNICATION_LOST,
	SAFE_DCDC_TEMP_STOP,
	SAFE_DCDC_COMMUNICATION_TIMEOUT,
	SAFE_DCDC_HIGH_IN_VOLT,
	SAFE_DCDC_HIGH_OUT_VOLT,
	SAFE_DCDC_OUT_SHORTCIRCUIT,
	SAFE_FUEL_INLET_PRESSURE_LOWER_THAN_AIR,
	SAFE_CELL_VOLTAGE_LOW,
	SAFE_CELL_VOLTAGE_CRITICALLY_LOW,
	SAFE_CELL_VOLTAGE_HIGH,
	SAFE_SENSOR_TEMP_COOL_ANODE_HIGH,
	SAFE_SENSOR_TEMP_COOL_ANODE_CRITICAL_HIGH,
	SAFE_SENSOR_TEMP_COOL_CATHODE_HIGH,
	SAFE_SENSOR_TEMP_COOL_CATHODE_CRITICAL_HIGH,
	SAFE_SENSOR_PRESSURE_ANODE_CRITICAL,
	SAFE_SENSOR_PRESSURE_CATHODE_CRITICAL,
	SAFE_FUEL_PRESSURE_LOWER_THAN_AIR,
	SAFE_SENSOR_TEMP_ANODE_LOST,
	SAFE_SENSOR_TEMP_CATHODE_LOST,
	SAFE_SAFETY_THREAD_STARTUP_PREREQ_ERROR
} FCSafetyFlags;

typedef enum {
  SAFETY_STATE_INIT = 0,
  SAFETY_STATE_READY,
  SAFETY_STATE_WARN,
  SAFETY_STATE_FAULT,
  SAFETY_STATE_STARTUP,
  SAFETY_STATE_RUN,
  SAFETY_STATE_SHUTDOWN,

} FCSafetyState;

// Fill fields with random data (for testing)
void fillFieldsRandom(void* data, FieldDescriptor_t* fields, size_t fieldCount);

// Unpack fields from a BridgeComSTM32 package
void unpackFieldsFromCom(CaptainInterFace* com, void* data, FieldDescriptor_t* fields, size_t fieldCount);

// Helper function for returning flag 
bool IsSafetyFlagActive(uint64_t flags, FCSafetyFlags e);

void mergeStructByFields(void* dst,
                         FieldDescriptor_t* dstFields, size_t dstCount,
                         const void* src,
                         FieldDescriptor_t* srcFields, size_t srcCount);


bool setFieldByName(void* data, FieldDescriptor_t* fields, size_t fieldCount, const char* name, float value);
bool getFieldByName(void* data, FieldDescriptor_t* fields, size_t fieldCount, const char* name, char* outStr, size_t outStrLen);